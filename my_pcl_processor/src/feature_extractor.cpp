#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>
#include <vector>
#include <string>
#include <chrono> // For the timer

// Include the *exact same* features.h file
#include "my_pcl_processor/features.h" 

// Define the point type
typedef pcl::PointXYZRGB PointT;

using namespace std::chrono_literals;

class FeatureExtractorNode : public rclcpp::Node
{
public:
    FeatureExtractorNode() : Node("feature_extractor_node"), samples_taken_(0)
    {
        // --- Parameters ---
        this->declare_parameter<std::string>("model_name", "default_model");
        this->declare_parameter<int>("max_samples", 5);
        this->declare_parameter<double>("capture_interval_sec", 2.0);

        model_name_ = this->get_parameter("model_name").as_string();
        max_samples_ = this->get_parameter("max_samples").as_int();
        double capture_interval_sec = this->get_parameter("capture_interval_sec").as_double();

        if (model_name_ == "default_model")
        {
            RCLCPP_ERROR(this->get_logger(), "Model name parameter 'model_name' not set!");
            rclcpp::shutdown();
            return;
        }

    
        std::string home = getenv("HOME");
        output_file_ = home + "/training.sav";

   
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/processed/downsampled",
            10,
            std::bind(&FeatureExtractorNode::cloudUpdateCallback, this, std::placeholders::_1));

        auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(capture_interval_sec));
            
        timer_ = this->create_wall_timer(
            interval,
            std::bind(&FeatureExtractorNode::processAndSaveFeatures, this));

        RCLCPP_INFO(this->get_logger(), "Feature Extractor node ready.");
        RCLCPP_INFO(this->get_logger(), "Saving %d samples for model: %s", max_samples_, model_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Capturing one sample every %.1f seconds...", capture_interval_sec);
        RCLCPP_INFO(this->get_logger(), "Move the object in Gazebo between captures!");
    }

    /**
     * @brief Subscriber callback. Just stores the latest message.
     */
    void cloudUpdateCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cluster_msg)
    {
        // Store the latest message
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_cloud_msg_ = cluster_msg;
    }

    /**
     * @brief Timer callback. This does all the work.
     */
    void processAndSaveFeatures()
    {
        // Check if we are done
        if (samples_taken_ >= max_samples_)
        {
            if(!shutdown_logged_) {
                RCLCPP_INFO(this->get_logger(), "All %d samples captured. Stopping.", max_samples_);
                timer_->cancel(); // Stop the timer
                shutdown_logged_ = true;
            }
            return;
        }

        sensor_msgs::msg::PointCloud2::SharedPtr msg_to_process;
        
        // Atomically get the latest cloud and clear the member
        {
            std::lock_guard<std::mutex> lock(cloud_mutex_);
            if (!latest_cloud_msg_)
            {
                RCLCPP_WARN(this->get_logger(), "Timer fired, but no point cloud received yet.");
                return;
            }
            msg_to_process = latest_cloud_msg_;
            latest_cloud_msg_ = nullptr; // Mark as "used"
        }

        RCLCPP_INFO(this->get_logger(), "Capturing sample %d of %d...", (samples_taken_ + 1), max_samples_);

        // 1. Convert ROS msg to PCL
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg_to_process, *cluster);

        if (cluster->empty()) {
            RCLCPP_WARN(this->get_logger(), "Cloud is empty, skipping sample.");
            return;
        }

        // 2. Compute Color Histogram
        std::vector<float> color_hist = computeColorHistogram(cluster);

        // 3. Compute Normals
        pcl::PointCloud<pcl::Normal>::Ptr normals = getNormals(cluster);

        // 4. Compute Normal Histogram
        std::vector<float> normal_hist = computeNormalHistogram(normals);

        // 5. Concatenate features
        std::vector<float> feature_vector;
        feature_vector.insert(feature_vector.end(), color_hist.begin(), color_hist.end());
        feature_vector.insert(feature_vector.end(), normal_hist.begin(), normal_hist.end());

        // 6. Save to file
        saveFeatures(feature_vector);

        // 7. Increment counter
        samples_taken_++;
    }

    void saveFeatures(const std::vector<float>& features)
    {
        std::ofstream outfile;
        // Open in append mode
        outfile.open(output_file_, std::ios_base::app); 
        
        if (!outfile.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open training file: %s", output_file_.c_str());
            return;
        }

        // Write the label (model name)
        outfile << model_name_;

        // Write all features
        for (const auto& feature : features)
        {
            outfile << "," << feature;
        }
        outfile << std::endl; // New line for next entry

        outfile.close();
        RCLCPP_INFO(this->get_logger(), "Successfully saved sample %d for %s.", samples_taken_ + 1, model_name_.c_str());
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_msg_;
    std::mutex cloud_mutex_;

    std::string model_name_;
    std::string output_file_;
    int samples_taken_;
    int max_samples_;
    bool shutdown_logged_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureExtractorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}