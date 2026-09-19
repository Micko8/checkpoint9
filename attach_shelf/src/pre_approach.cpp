#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <chrono>
#include <sensor_msgs/msg/laser_scan.hpp>

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("preapproach_node") {

    RCLCPP_INFO(this->get_logger(), "Preapproach : Constructor");
  }

private:
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}