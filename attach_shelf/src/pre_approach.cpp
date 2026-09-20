#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include <algorithm>
#include <chrono>
#include <sensor_msgs/msg/laser_scan.hpp>

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("preapproach_node") {

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

    is_moving_ = true; // To be put somewhere else

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",
                                                                   qos_profile);

    auto timer_period = std::chrono::milliseconds(100);
    timer_ = this->create_wall_timer(
        timer_period, std::bind(&PreApproach::timer_callback, this));

    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos_profile,
        std::bind(&PreApproach::laser_callback, this, std::placeholders::_1));
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!is_moving_) {
      return;
    }

    // Only check straight ahead
    float front = msg->ranges[99];

    RCLCPP_INFO(this->get_logger(), "front=%.2f", front);
    is_moving_ = true;

    if (front < obstacle_) {
      /*
            // Snapshot yaw at turn start
            yaw_at_turn_start_ = yaw_;

      */
      /*RCLCPP_INFO(this->get_logger(),
                  "Front wall! max_idx=%d direction=%.2f yaw_start=%.2f",
                  max_idx, direction_, yaw_at_turn_start_);*/

      RCLCPP_INFO(this->get_logger(), "Front wall detected !");
      is_moving_ = false;
    }
  }

  void timer_callback() {
    auto msg = geometry_msgs::msg::Twist();

    if (is_moving_) {
#if 0
      // Normalisation dans [-pi, pi] : sans elle, un cap qui traverse +/-pi
      // fait sauter la difference a +/-2pi et le virage s'arrete aussitot.
      double delta = yaw_ - yaw_at_turn_start_;
      double turned_so_far = std::atan2(std::sin(delta), std::cos(delta));

      if (std::abs(turned_so_far) < std::abs(direction_) - 0.1) {
        msg.linear.x = 0.1;
        msg.angular.z = direction_ / 2;
        RCLCPP_INFO(this->get_logger(), "Turning... turned=%.2f target=%.2f",
                    turned_so_far, direction_);
      } else {
        is_turning_ = false;
        RCLCPP_INFO(this->get_logger(), "Turn complete.");
      }
#endif
      RCLCPP_INFO(this->get_logger(), "Moving Forward");
      msg.linear.x = 0.5;
      msg.angular.z = 0.0;
    } else {
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
    }

    publisher_->publish(msg);
  }

  /*
  laser_scan_ = *msg;
  laser_data_received_ = true;
*/

  float obstacle_;
  int degrees_;
  bool is_moving_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}