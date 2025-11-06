#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <chrono>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> 

class PcdSaverNode : public rclcpp::Node
{
public:
    PcdSaverNode() : Node("pcd_saver_node")
    {

        rclcpp::sleep_for(std::chrono::seconds(2));
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();


        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/processed/passthrough", 
            qos,
            std::bind(&PcdSaverNode::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "PCD Saver Node has started.");
        RCLCPP_INFO(this->get_logger(), "Waiting for one message from /processed/passthrough...");
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message!");


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        RCLCPP_INFO(this->get_logger(), "Converted message to PCL (PointXYZRGB).");

        std::string save_path = "snapshot.pcd";
        

        pcl::io::savePCDFileASCII(save_path, *cloud);

        RCLCPP_INFO(this->get_logger(), "Successfully saved cloud with %ld points to %s", cloud->size(), save_path.c_str());


        RCLCPP_INFO(this->get_logger(), "Shutting down node.");
        rclcpp::shutdown();
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PcdSaverNode>());
    rclcpp::shutdown();
    return 0;
}