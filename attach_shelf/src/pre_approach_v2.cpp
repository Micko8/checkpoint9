#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "custom_interface/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using GoToLoading = custom_interface::srv::GoToLoading;

class PreApproachV2 : public rclcpp::Node {
public:
  PreApproachV2() : Node("pre_approach_v2_node") {
    this->declare_parameter<double>("obstacle", 0.0);
    this->declare_parameter<int>("degrees", 0);
    this->declare_parameter<bool>("final_approach", false);

    obstacle_ = this->get_parameter("obstacle").as_double();
    degrees_ = this->get_parameter("degrees").as_int();
    final_approach_ = this->get_parameter("final_approach").as_bool();

    RCLCPP_INFO(this->get_logger(),
                "Pre-approach ready: obstacle=%.2f, degrees=%d, "
                "final_approach=%s",
                obstacle_, degrees_, final_approach_ ? "true" : "false");

    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    velocity_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",
                                                          qos_profile);

    laser_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos_profile,
            std::bind(&PreApproachV2::laser_callback, this,
                      std::placeholders::_1));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos_profile,
        std::bind(&PreApproachV2::odom_callback, this,
                  std::placeholders::_1));

    approach_client_ = this->create_client<GoToLoading>("/approach_shelf");

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PreApproachV2::control_callback, this));
  }

private:
  enum class State {
    MOVING_TO_WALL,
    TURNING,
    WAITING_FOR_SERVICE,
    FINAL_APPROACH_RUNNING,
    FINISHED
  };

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (!laser_initialized_) {
      RCLCPP_INFO(this->get_logger(), "Laser initialized");
      laser_initialized_ = true;
    }

    if (state_ != State::MOVING_TO_WALL || scan->ranges.empty() ||
        scan->angle_increment == 0.0) {
      return;
    }

    const int front_index =
        static_cast<int>(-scan->angle_min / scan->angle_increment);

    if (front_index < 0 ||
        front_index >= static_cast<int>(scan->ranges.size())) {
      return;
    }

    const float front_distance = scan->ranges[front_index];

    if (front_distance < obstacle_) {
      yaw_at_turn_start_ = yaw_;
      target_yaw_ =
          yaw_at_turn_start_ + (static_cast<double>(degrees_) * M_PI / 180.0);
      state_ = State::TURNING;

      RCLCPP_INFO(this->get_logger(),
                  "Wall detected at %.3f m; turning from %.3f to %.3f rad",
                  front_distance, yaw_at_turn_start_, target_yaw_);
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    tf2::Quaternion quaternion(
        odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    tf2::Matrix3x3 rotation_matrix(quaternion);
    double roll = 0.0;
    double pitch = 0.0;
    rotation_matrix.getRPY(roll, pitch, yaw_);
  }

  void control_callback() {
    if (state_ == State::WAITING_FOR_SERVICE) {
      try_start_final_approach();
      return;
    }

    if (state_ == State::FINAL_APPROACH_RUNNING) {
      // The pre-approach node has handed ownership of /cmd_vel to the server.
      return;
    }

    if (state_ == State::FINISHED) {
      shutdown_node();
      return;
    }

    geometry_msgs::msg::Twist command;

    if (state_ == State::MOVING_TO_WALL) {
      command.linear.x = 0.5;
    } else if (state_ == State::TURNING) {
      control_turn(command);
    }

    velocity_publisher_->publish(command);

    if (state_ == State::WAITING_FOR_SERVICE || state_ == State::FINISHED) {
      velocity_publisher_.reset();
      RCLCPP_INFO(this->get_logger(),
                  "Released ownership of /cmd_vel");
    }

    if (state_ == State::FINISHED) {
      shutdown_node();
    }
  }

  void control_turn(geometry_msgs::msg::Twist &command) {
    double yaw_error = normalize_angle(target_yaw_ - yaw_);

    if (std::abs(yaw_error) <= 0.05 || degrees_ == 0) {
      RCLCPP_INFO(this->get_logger(), "Pre-approach rotation complete");

      // The zero command is published by control_callback after this method
      // returns. No further velocity command is published by this node.
      if (final_approach_) {
        state_ = State::WAITING_FOR_SERVICE;
        service_call_not_before_ =
            std::chrono::steady_clock::now() + handoff_delay_;
        RCLCPP_INFO(this->get_logger(),
                    "Waiting to call /approach_shelf");
      } else {
        state_ = State::FINISHED;
        RCLCPP_INFO(this->get_logger(),
                    "final_approach=false: stopping after pre-approach");
      }
      return;
    }

    command.angular.z = degrees_ < 0 ? -0.5 : 0.5;
  }

  void try_start_final_approach() {
    if (std::chrono::steady_clock::now() < service_call_not_before_) {
      return;
    }

    if (!approach_client_->service_is_ready()) {
      if (!service_wait_reported_) {
        RCLCPP_INFO(this->get_logger(),
                    "Service /approach_shelf is not ready yet");
        service_wait_reported_ = true;
      }
      return;
    }

    auto request = std::make_shared<GoToLoading::Request>();
    request->attach_to_shelf = true;
    state_ = State::FINAL_APPROACH_RUNNING;

    RCLCPP_INFO(this->get_logger(), "Calling /approach_shelf");

    approach_client_->async_send_request(
        request,
        std::bind(&PreApproachV2::service_response_callback, this,
                  std::placeholders::_1));
  }

  void service_response_callback(
      rclcpp::Client<GoToLoading>::SharedFuture response_future) {
    const auto response = response_future.get();

    if (response->complete) {
      RCLCPP_INFO(this->get_logger(),
                  "Final approach and lifting completed successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Final approach failed");
    }

    state_ = State::FINISHED;
    shutdown_node();
  }

  void shutdown_node() {
    if (shutdown_requested_) {
      return;
    }

    shutdown_requested_ = true;
    RCLCPP_INFO(this->get_logger(), "Stopping pre_approach_v2 node");
    rclcpp::shutdown();
  }

  double normalize_angle(double angle) const {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  double obstacle_{0.0};
  int degrees_{0};
  bool final_approach_{false};
  bool laser_initialized_{false};
  bool service_wait_reported_{false};
  bool shutdown_requested_{false};
  double yaw_{0.0};
  double yaw_at_turn_start_{0.0};
  double target_yaw_{0.0};
  State state_{State::MOVING_TO_WALL};
  std::chrono::steady_clock::time_point service_call_not_before_{};
  const std::chrono::milliseconds handoff_delay_{500};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      velocity_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Client<GoToLoading>::SharedPtr approach_client_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproachV2>());
  rclcpp::shutdown();
  return 0;
}
