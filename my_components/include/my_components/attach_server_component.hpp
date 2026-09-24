#ifndef COMPOSITION__ATTACHSERVER_COMPONENT_HPP_
#define COMPOSITION__ATTACHSERVER_COMPONENT_HPP_

#include "my_components/visibility_control.h"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "custom_interface/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace my_components {

using GoToLoading = custom_interface::srv::GoToLoading;

class AttachServer : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachServer(const rclcpp::NodeOptions &options);

private:
  enum class MotionState {
    IDLE,
    APPROACHING_CART,
    MOVING_FORWARD,
    LIFTING,
    COMPLETE
  };

  // Callbacks
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void control_callback();
  void handle_request(const std::shared_ptr<GoToLoading::Request> request,
                      std::shared_ptr<GoToLoading::Response> response);

  // Helpers
  bool find_leg_indices(const std::vector<float> &intensities,
                        std::size_t &first_leg_index,
                        std::size_t &second_leg_index) const;
  bool valid_range(const sensor_msgs::msg::LaserScan &scan,
                   std::size_t index) const;
  double limit(double value, double maximum_absolute_value) const;
  void publish_cart_frame(const sensor_msgs::msg::LaserScan &scan);

  // Motion control
  void control_approach(geometry_msgs::msg::Twist &command);
  void control_forward_motion(geometry_msgs::msg::Twist &command);
  void command_lift();

  // ROS interfaces
  rclcpp::Service<GoToLoading>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  double intensity_threshold_{8000.0};
  int minimum_leg_separation_{10};

  // State
  std::atomic<bool> scan_received_{false};
  std::atomic<bool> legs_detected_{false};
  std::atomic<bool> approach_requested_{false};
  bool missing_intensities_reported_{false};
  bool odom_received_{false};
  bool tf_error_reported_{false};
  double cart_x_{0.0};
  double cart_y_{0.0};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double forward_start_x_{0.0};
  double forward_start_y_{0.0};
  std::size_t last_first_leg_index_{0};
  std::size_t last_second_leg_index_{0};

  std::atomic<MotionState> motion_state_{MotionState::IDLE};
  std::atomic<bool> lifting_done_{false};
  std::atomic<bool> lift_command_sent_{false};
  std::chrono::steady_clock::time_point lift_command_time_{};

  // Constants
  const std::chrono::seconds mission_timeout_{120};
  const std::chrono::seconds lift_wait_duration_{1};
  const std::string robot_base_frame_{"robot_base_link"};
  const double cart_position_tolerance_{0.05};
  const double approach_linear_speed_{0.12};
  const double angular_gain_{1.5};
  const double maximum_angular_speed_{0.5};
  const double maximum_driving_angle_{0.25};
  const double final_forward_distance_{0.35};
  const double final_forward_speed_{0.10};
};

} // namespace my_components

#endif // COMPOSITION__ATTACHSERVER_COMPONENT_HPP_
