#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class PreApproach : public rclcpp::Node {
public:
  PreApproach()
      : Node("preapproach_node"), mission_complete_(false), is_moving_(true),
        is_turning_(false), laser_initialized_(false), yaw_(0.0),
        yaw_at_turn_start_(0.0), target_yaw_(0.0) {

    RCLCPP_INFO(this->get_logger(), "Preapproach : Constructor");

    // Declare parameters with default values
    this->declare_parameter<double>("obstacle", 0.0);
    this->declare_parameter<int>("degrees", 0);

    // Read parameter values
    obstacle_ = this->get_parameter("obstacle").as_double();
    degrees_ = this->get_parameter("degrees").as_int();

    RCLCPP_INFO(this->get_logger(), "obstacle: %.2f", obstacle_);
    RCLCPP_INFO(this->get_logger(), "degrees: %d", degrees_);

    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",
                                                                   qos_profile);

    auto timer_period = std::chrono::milliseconds(100);
    timer_ = this->create_wall_timer(
        timer_period, std::bind(&PreApproach::timer_callback, this));

    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos_profile,
        std::bind(&PreApproach::laser_callback, this, std::placeholders::_1));

    subscriber_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos_profile,
        std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

    mission_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PreApproach::check_mission_complete, this));
  }

private:
  void check_mission_complete() {
    if (!is_turning_ && !is_moving_ && mission_complete_) {
      RCLCPP_INFO(this->get_logger(), "Mission complete! Shutting down...");
      rclcpp::shutdown();
    }
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // Just log initialization once
    if (!laser_initialized_) {
      RCLCPP_INFO(this->get_logger(), "Laser initialized");
      laser_initialized_ = true;
    }

    if (!is_moving_) {
      return;
    }

    // Calcule l'index qui pointe droit devant
    int front_index = static_cast<int>(-msg->angle_min / msg->angle_increment);
    float front = msg->ranges[front_index];

    RCLCPP_INFO(this->get_logger(), "Front index: %d, front=%.2f", front_index,
                front);

    is_moving_ = true;

    if (front < obstacle_) {
      RCLCPP_INFO(this->get_logger(), "Front wall detected !");
      is_moving_ = false;
      yaw_at_turn_start_ = yaw_;
      target_yaw_ = std::round(yaw_at_turn_start_ + (degrees_ * M_PI / 180.0));
      RCLCPP_INFO(this->get_logger(),
                  "Starting rotation: from %.3f to %.3f rad",
                  yaw_at_turn_start_, target_yaw_);
      is_turning_ = true;
    }
  }

  void timer_callback() {
    auto msg = geometry_msgs::msg::Twist();

    if (!is_turning_ && is_moving_) {
      RCLCPP_INFO(this->get_logger(), "Moving Forward");
      msg.linear.x = 0.5;
      msg.angular.z = 0.0;
    } else if (is_turning_) {
      msg.linear.x = 0.0;

      if (degrees_ != 0.0) {
        // Rotate in direction of degrees_
        msg.angular.z = (degrees_ < 0.0) ? -0.5 : 0.5;

        // Calculate angle difference (handle wrap-around)
        double yaw_diff = target_yaw_ - yaw_;

        // Normalize to [-pi, pi]
        while (yaw_diff > M_PI)
          yaw_diff -= 2 * M_PI;
        while (yaw_diff < -M_PI)
          yaw_diff += 2 * M_PI;

        RCLCPP_INFO(this->get_logger(),
                    "Rotating, yaw_diff: %.3f rad (%.1f deg)", yaw_diff,
                    yaw_diff * 180 / M_PI);

        // Stop when close enough (tolerance ~0.05 rad ≈ 3°)
        if (std::abs(yaw_diff) < 0.05) {
          is_turning_ = false;
          mission_complete_ = true; // ← Flag
          RCLCPP_INFO(this->get_logger(), "Rotation Complete.");
        }
      } else {
        msg.angular.z = 0.0;
      }

    } else {
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Robot Stopped.");
    }

    publisher_->publish(msg);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw_);
  }

  float obstacle_;
  int degrees_;
  bool is_moving_;
  bool laser_initialized_;
  double yaw_;
  double yaw_at_turn_start_;
  double target_yaw_;
  bool is_turning_;
  bool mission_complete_;
  rclcpp::TimerBase::SharedPtr mission_timer_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}
