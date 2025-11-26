#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <vector>
#include <string>
#include <memory>

#include "my_pcl_processor/features.h" 

#include "feature_extractor_pkg/srv/feature_extraction.hpp"

typedef pcl::PointXYZRGB PointT;
using FeatureExtraction = feature_extractor_pkg::srv::FeatureExtraction;

class FeatureExtractorNode : public rclcpp::Node
{
public:
    FeatureExtractorNode() : Node("feature_extractor_for_model_node")
    {
        server_ = this->create_service<FeatureExtraction>(
            "feature_extraction_service",
            std::bind(&FeatureExtractorNode::featureExtractionCallback, this, 
                      std::placeholders::_1, 
                      std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "*********Feature Extractor Service is running for (model)*********.");
     }

private:
    rclcpp::Service<FeatureExtraction>::SharedPtr server_;

    void featureExtractionCallback(
        const std::shared_ptr<FeatureExtraction::Request> request,
        std::shared_ptr<FeatureExtraction::Response> response){
        RCLCPP_INFO(this->get_logger(), "Received Feature Extraction Request.");
        
        pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(request->input_cluster, *cluster); 
        
        if (cluster->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty cloud, skipping sample.");
            response->feature_vector.clear();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "No of points in the cluster: %ld", cluster->size());

        std::vector<float> color_hist = computeColorHistogram(cluster);
        pcl::PointCloud<pcl::Normal>::Ptr normals = getNormals(cluster);
        std::vector<float> normal_hist = computeNormalHistogram(normals);

        std::vector<float> feature_vector;
        feature_vector.insert(feature_vector.end(), color_hist.begin(), color_hist.end());
        feature_vector.insert(feature_vector.end(), normal_hist.begin(), normal_hist.end());

        response->feature_vector = feature_vector;

        RCLCPP_INFO(this->get_logger(), "Sent response with %zu features.", response->feature_vector.size());
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureExtractorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}