#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_ros/transforms.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class CloudTransformer : public rclcpp::Node
{
public:
  CloudTransformer() : Node("cloud_transformer"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/points", 10,
        std::bind(&CloudTransformer::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/sensor_stick/transformed_world", 10);

    RCLCPP_INFO(this->get_logger(), "Cloud Transformer started...");
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    try
    {
      sensor_msgs::msg::PointCloud2 output;
      pcl_ros::transformPointCloud("world", *msg, output, tf_buffer_);
      output.header.frame_id = "world";
      pub_->publish(output);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Transform unavailable: %s", ex.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudTransformer>());
  rclcpp::shutdown();
  return 0;
}
