#include "laser_geometry/laserscan_to_pointcloud.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

using namespace laser_geometry;

LaserToPointCloud::LaserToPointCloud(const rclcpp::NodeOptions & options)
: Node("laser_to_pointcloud", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
    this->declare_parameter("fixed_frame", "odom");
    fixed_frame_ = this->get_parameter("fixed_frame").as_string();
    
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 1, std::bind(&LaserToPointCloud::laserScanCallback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
    
    RCLCPP_INFO(this->get_logger(), "Using fixed frame: %s", fixed_frame_.c_str());
}

void LaserToPointCloud::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    sensor_msgs::msg::PointCloud2 cloud;
    try {
        projector_.transformLaserScanToPointCloud(fixed_frame_, *msg, cloud, tf_buffer_);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_DEBUG(this->get_logger(), "T1 - Transform error: %s", ex.what());
        return;
    }

    try {
        auto transform = tf_buffer_.lookupTransform(msg->header.frame_id, fixed_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.01));
        tf2::doTransform(cloud, cloud, transform);
        cloud.header.frame_id = msg->header.frame_id;  
        publisher_->publish(cloud);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_DEBUG(this->get_logger(), "T2 - Transform error: %s", ex.what());
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(laser_geometry::LaserToPointCloud)