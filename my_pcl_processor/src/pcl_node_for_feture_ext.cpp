#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>



class PclProcessingNode : public rclcpp::Node
{
public:
    PclProcessingNode() : Node("pcl_processing_node")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor_stick/transformed_world",
            qos,
            std::bind(&PclProcessingNode::topic_callback, this, std::placeholders::_1));

        downsampled_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/processed/downsampled", 10);
        passthrough_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/processed/passthrough", 10);

        RCLCPP_INFO(this->get_logger(), "PCL Processing Node has started.");
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);
        
        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty point cloud.");
            return;
        }

    
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.01f, 0.01f, 0.01f); 
        vg.filter(*cloud_downsampled);

        publish_cloud(cloud_downsampled, msg->header, downsampled_pub_);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passed(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(cloud_downsampled);
        pass.setFilterFieldName("z");        
        pass.setFilterLimits(0.09935, 0.575935);    
        pass.filter(*cloud_passed);

        publish_cloud(cloud_passed, msg->header, passthrough_pub_);
    }

    void publish_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                       const std_msgs::msg::Header& header,
                       const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher)
    {
        if (cloud->empty()) return;
        
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud, output_msg);
        output_msg.header.frame_id = header.frame_id;
        output_msg.header.stamp = this->get_clock()->now();
        publisher->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr passthrough_pub_;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PclProcessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}