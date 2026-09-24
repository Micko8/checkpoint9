#ifndef COMPOSITION__PREAPPROACH_COMPONENT_HPP_
#define COMPOSITION__PREAPPROACH_COMPONENT_HPP_

#include "my_components/visibility_control.h"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include <sensor_msgs/msg/laser_scan.hpp>

namespace my_components {

class PreApproach : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit PreApproach(const rclcpp::NodeOptions &options);

private:
  // Callbacks
  void check_mission_complete();
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void timer_callback();
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Parameters
  float obstacle_;
  int degrees_;

  // State
  bool mission_complete_;
  bool is_moving_;
  bool is_turning_;
  bool laser_initialized_;
  double yaw_;
  double yaw_at_turn_start_;
  double target_yaw_;

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr mission_timer_;
};

} // namespace my_components

#endif // COMPOSITION__PREAPPROACH_COMPONENT_HPP_
