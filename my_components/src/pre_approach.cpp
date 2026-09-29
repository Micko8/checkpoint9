#include "my_components/pre_approach_component.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace my_components {
namespace {
constexpr float kObstacleDistance = 0.3f;
constexpr int kTurnDegrees = -90;
} // namespace

PreApproach::PreApproach(const rclcpp::NodeOptions &options)
    : Node("pre_approach_node", options), mission_complete_(false),
      is_moving_(true), is_turning_(false), laser_initialized_(false),
      yaw_(0.0), yaw_at_turn_start_(0.0), target_yaw_(0.0) {

  RCLCPP_INFO(this->get_logger(), "Preapproach : Constructor");
  RCLCPP_INFO(this->get_logger(), "obstacle: %.2f", kObstacleDistance);
  RCLCPP_INFO(this->get_logger(), "degrees: %d", kTurnDegrees);

  publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&PreApproach::timer_callback, this));

  subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&PreApproach::laser_callback, this, std::placeholders::_1));

  subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

  mission_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PreApproach::check_mission_complete, this));
}

void PreApproach::check_mission_complete() {
  if (!is_turning_ && !is_moving_ && mission_complete_) {
    RCLCPP_INFO(this->get_logger(), "Mission complete!");
    // Dans un composant on ne fait pas rclcpp::shutdown() (Ã§a arrÃªterait tout
    // le container) : on arrÃªte simplement les timers.
    // On publie une derniÃšre commande nulle : sinon le robot garderait la
    // derniÃšre vitesse reÃ§ue (AttachServer prend ensuite le relais sur
    // /cmd_vel).
    publisher_->publish(geometry_msgs::msg::Twist());
    mission_timer_->cancel();
    timer_->cancel();
  }
}

void PreApproach::laser_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  if (!laser_initialized_) {
    RCLCPP_INFO(this->get_logger(), "Laser initialized");
    laser_initialized_ = true;
  }

  if (!is_moving_) {
    return;
  }

  if (msg->ranges.empty() || msg->angle_increment == 0.0) {
    return;
  }

  const int front_index =
      static_cast<int>(-msg->angle_min / msg->angle_increment);

  if (front_index < 0 || front_index >= static_cast<int>(msg->ranges.size())) {
    return;
  }

  const float front = msg->ranges[front_index];

  if (!std::isfinite(front)) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Front index: %d, front=%.2f", front_index,
              front);

  is_moving_ = true;

  if (front < kObstacleDistance) {
    RCLCPP_INFO(this->get_logger(), "Front wall detected !");
    is_moving_ = false;
    yaw_at_turn_start_ = yaw_;
    target_yaw_ = yaw_at_turn_start_ + (kTurnDegrees * M_PI / 180.0);
    RCLCPP_INFO(this->get_logger(), "Starting rotation: from %.3f to %.3f rad",
                yaw_at_turn_start_, target_yaw_);
    is_turning_ = true;
  }
}

void PreApproach::timer_callback() {
  auto msg = geometry_msgs::msg::Twist();

  if (!is_turning_ && is_moving_) {
    RCLCPP_INFO(this->get_logger(), "Moving Forward");
    msg.linear.x = 0.5;
    msg.angular.z = 0.0;
  } else if (is_turning_) {
    msg.linear.x = 0.0;

    double yaw_diff = target_yaw_ - yaw_;

    while (yaw_diff > M_PI) {
      yaw_diff -= 2.0 * M_PI;
    }
    while (yaw_diff < -M_PI) {
      yaw_diff += 2.0 * M_PI;
    }

    RCLCPP_INFO(this->get_logger(), "Rotating, yaw_diff: %.3f rad (%.1f deg)",
                yaw_diff, yaw_diff * 180.0 / M_PI);

    if (std::abs(yaw_diff) < 0.05) {
      msg.angular.z = 0.0;
      is_turning_ = false;
      mission_complete_ = true;
      RCLCPP_INFO(this->get_logger(), "Rotation Complete.");
    } else {
      msg.angular.z = (kTurnDegrees < 0) ? -0.5 : 0.5;
    }

  } else {
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "Robot Stopped.");
  }

  publisher_->publish(msg);
}

void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, yaw_);
}

} // namespace my_components

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)
