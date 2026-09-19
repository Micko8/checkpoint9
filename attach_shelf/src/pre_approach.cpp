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

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",
                                                                   qos_profile);

    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos_profile,
        std::bind(&PreApproach::laser_callback, this, std::placeholders::_1));
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    /*
    laser_scan_ = *msg;
    laser_data_received_ = true;
*/
  }
  float obstacle_;
  int degrees_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}