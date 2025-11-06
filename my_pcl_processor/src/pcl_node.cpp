#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// PCL filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

// PCL segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>

// PCL k-d
#include <pcl/kdtree/kdtree.h>


class PclProcessingNode : public rclcpp::Node
{
public:
    PclProcessingNode() : Node("pcl_processing_node")
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/points",
            qos,
            std::bind(&PclProcessingNode::topic_callback, this, std::placeholders::_1));

        downsampled_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/processed/downsampled", 10);
        passthrough_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/processed/passthrough", 10);
        plane_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/processed/plane", 10);
        clusters_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/processed/clusters", 10);

        RCLCPP_INFO(this->get_logger(), "PCL Processing Node has started.");
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS msg to PCL data
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);
        
        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty point cloud.");
            return;
        }

        // Voxel Grid Downsampling
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.01f, 0.01f, 0.01f); 
        vg.filter(*cloud_downsampled);

        publish_cloud(cloud_downsampled, msg->header, downsampled_pub_);

        // PassThrough Filter
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passed(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(cloud_downsampled);
        pass.setFilterFieldName("z");        
        pass.setFilterLimits(0.09935, 0.575935);    
        pass.filter(*cloud_passed);

        publish_cloud(cloud_passed, msg->header, passthrough_pub_);


        // RANSAC Plane Segmentation
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01); 
        seg.setInputCloud(cloud_passed);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
            return;
        }

   
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud_passed);
        extract.setIndices(inliers);


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.setNegative(false);
        extract.filter(*cloud_plane);
        publish_cloud(cloud_plane, msg->header, plane_pub_);

    
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_objects(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.setNegative(true);
        extract.filter(*cloud_objects);

        if (cloud_objects->empty()) {
            RCLCPP_INFO(this->get_logger(), "No objects found after plane removal.");
            return;
        }

        
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(cloud_objects);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(0.02);    
        ec.setMinClusterSize(100);       
        ec.setMaxClusterSize(25000);    
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_objects);
        ec.extract(cluster_indices);

        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusters(new pcl::PointCloud<pcl::PointXYZRGB>);

        int cluster_id = 0;
        for (const auto& cluster : cluster_indices)
        {
            uint8_t r = rand() % 256;
            uint8_t g = rand() % 256;
            uint8_t b = rand() % 256;

            for (const auto& idx : cluster.indices)
            {
                pcl::PointXYZRGB pt = (*cloud_objects)[idx];
                
                pcl::PointXYZRGB color_pt;
                color_pt.x = pt.x;
                color_pt.y = pt.y;
                color_pt.z = pt.z;
                color_pt.r = r;
                color_pt.g = g;
                color_pt.b = b;
                
                cloud_clusters->push_back(color_pt);
            }
            cluster_id++;
        }

        RCLCPP_INFO(this->get_logger(), "Found %d clusters.", cluster_id);

        if (!cloud_clusters->empty()) {
            sensor_msgs::msg::PointCloud2 output_clusters;
            pcl::toROSMsg(*cloud_clusters, output_clusters);
            output_clusters.header.frame_id = msg->header.frame_id;
            output_clusters.header.stamp = this->get_clock()->now();
            clusters_pub_->publish(output_clusters);
        }
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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clusters_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PclProcessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}