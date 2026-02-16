#ifndef LASER_TO_POINTCLOUD_HPP
#define LASER_TO_POINTCLOUD_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace laser_geometry {
class LaserToPointCloud : public rclcpp::Node
{
public:
    LaserToPointCloud(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    laser_geometry::LaserProjection projector_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::string fixed_frame_;
};
}
#endif  // LASER_TO_POINTCLOUD_HPP