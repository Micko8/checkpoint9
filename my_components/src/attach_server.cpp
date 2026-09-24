#include "my_components/attach_server_component.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <thread>

#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace my_components {

AttachServer::AttachServer(const rclcpp::NodeOptions &options)
    : Node("attach_server", options) {
  this->declare_parameter<double>("intensity_threshold", 8000.0);
  this->declare_parameter<int>("minimum_leg_separation", 10);

  intensity_threshold_ = this->get_parameter("intensity_threshold").as_double();
  minimum_leg_separation_ =
      this->get_parameter("minimum_leg_separation").as_int();

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  velocity_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  elevator_publisher_ =
      this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);

  odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&AttachServer::odom_callback, this,
                std::placeholders::_1));

  scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&AttachServer::scan_callback, this,
                std::placeholders::_1));

  service_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  service_ = this->create_service<GoToLoading>(
      "/approach_shelf",
      std::bind(&AttachServer::handle_request, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_callback_group_);

  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&AttachServer::control_callback, this));

  RCLCPP_INFO(this->get_logger(),
              "Service /approach_shelf is ready "
              "(intensity threshold: %.0f, minimum separation: %d samples)",
              intensity_threshold_, minimum_leg_separation_);
}

void AttachServer::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  scan_received_ = true;
  legs_detected_ = false;

  if (scan->intensities.empty()) {
    if (!missing_intensities_reported_) {
      RCLCPP_WARN(this->get_logger(),
                  "The /scan message contains no intensity values");
      missing_intensities_reported_ = true;
    }
    return;
  }

  std::size_t first_leg_index = 0;
  std::size_t second_leg_index = 0;

  if (!find_leg_indices(scan->intensities, first_leg_index,
                        second_leg_index)) {
    return;
  }

  if (!valid_range(*scan, first_leg_index) ||
      !valid_range(*scan, second_leg_index)) {
    return;
  }

  const double first_angle =
      scan->angle_min + first_leg_index * scan->angle_increment;
  const double second_angle =
      scan->angle_min + second_leg_index * scan->angle_increment;

  const double first_x = scan->ranges[first_leg_index] * std::cos(first_angle);
  const double first_y = scan->ranges[first_leg_index] * std::sin(first_angle);
  const double second_x =
      scan->ranges[second_leg_index] * std::cos(second_angle);
  const double second_y =
      scan->ranges[second_leg_index] * std::sin(second_angle);

  cart_x_ = (first_x + second_x) / 2.0;
  cart_y_ = (first_y + second_y) / 2.0;
  legs_detected_ = true;

  if (approach_requested_) {
    publish_cart_frame(*scan);
  }

  if (first_leg_index != last_first_leg_index_ ||
      second_leg_index != last_second_leg_index_) {
    RCLCPP_INFO(this->get_logger(),
                "Shelf legs detected at indices %zu and %zu; "
                "center in laser frame: x=%.3f m, y=%.3f m",
                first_leg_index, second_leg_index, cart_x_, cart_y_);
    last_first_leg_index_ = first_leg_index;
    last_second_leg_index_ = second_leg_index;
  }
}

bool AttachServer::find_leg_indices(
    const std::vector<float> &intensities, std::size_t &first_leg_index,
    std::size_t &second_leg_index) const {
  bool first_leg_found = false;

  for (std::size_t index = 0; index < intensities.size(); ++index) {
    if (intensities[index] < intensity_threshold_) {
      continue;
    }

    if (!first_leg_found) {
      first_leg_index = index;
      first_leg_found = true;
      continue;
    }

    // Consecutive reflective indices belong to the same physical leg.
    if (index - first_leg_index >=
        static_cast<std::size_t>(minimum_leg_separation_)) {
      second_leg_index = index;
      return true;
    }
  }

  return false;
}

bool AttachServer::valid_range(const sensor_msgs::msg::LaserScan &scan,
                                        std::size_t index) const {
  if (index >= scan.ranges.size()) {
    return false;
  }

  const float range = scan.ranges[index];
  return std::isfinite(range) && range >= scan.range_min &&
         range <= scan.range_max;
}

void AttachServer::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr odom) {
  odom_x_ = odom->pose.pose.position.x;
  odom_y_ = odom->pose.pose.position.y;
  odom_received_ = true;
}

void AttachServer::control_callback() {
  geometry_msgs::msg::Twist command;

  if (motion_state_ == MotionState::APPROACHING_CART) {
    control_approach(command);
  } else if (motion_state_ == MotionState::MOVING_FORWARD) {
    control_forward_motion(command);
  } else if (motion_state_ == MotionState::LIFTING) {
    command_lift();
  }

  if (motion_state_ != MotionState::IDLE) {
    velocity_publisher_->publish(command);
  }
}

void AttachServer::control_approach(
    geometry_msgs::msg::Twist &command) {
  geometry_msgs::msg::TransformStamped robot_to_cart;

  try {
    robot_to_cart = tf_buffer_->lookupTransform(robot_base_frame_, "cart_frame",
                                                tf2::TimePointZero);
  } catch (const tf2::TransformException &exception) {
    if (!tf_error_reported_) {
      RCLCPP_WARN(this->get_logger(),
                  "Waiting for robot_base_link -> cart_frame TF: %s",
                  exception.what());
      tf_error_reported_ = true;
    }
    return;
  }

  tf_error_reported_ = false;

  const double target_x = robot_to_cart.transform.translation.x;
  const double target_y = robot_to_cart.transform.translation.y;
  const double distance = std::hypot(target_x, target_y);
  const double angle_error = std::atan2(target_y, target_x);

  if (distance <= cart_position_tolerance_) {
    if (!odom_received_) {
      return;
    }

    forward_start_x_ = odom_x_;
    forward_start_y_ = odom_y_;
    motion_state_ = MotionState::MOVING_FORWARD;
    RCLCPP_INFO(this->get_logger(),
                "cart_frame reached; starting final %.2f m motion",
                final_forward_distance_);
    return;
  }

  command.angular.z = limit(angular_gain_ * angle_error, maximum_angular_speed_);

  // First face the target. Moving while the angular error is large would
  // create a wide curve and make the controller less predictable.
  if (std::abs(angle_error) < maximum_driving_angle_) {
    command.linear.x = std::min(approach_linear_speed_, distance);
  }
}

void AttachServer::control_forward_motion(
    geometry_msgs::msg::Twist &command) {
  if (!odom_received_) {
    return;
  }

  const double travelled =
      std::hypot(odom_x_ - forward_start_x_, odom_y_ - forward_start_y_);

  if (travelled >= final_forward_distance_) {
    motion_state_ = MotionState::LIFTING;
    RCLCPP_INFO(this->get_logger(),
                "Final forward motion complete: travelled %.3f m", travelled);
    return;
  }

  command.linear.x = final_forward_speed_;
}

void AttachServer::command_lift() {
  if (!lift_command_sent_) {
    elevator_publisher_->publish(std_msgs::msg::String());
    lift_command_sent_ = true;
    lift_command_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "Lift command published on /elevator_up");
    return;
  }

  const auto lifting_duration =
      std::chrono::steady_clock::now() - lift_command_time_;

  if (lifting_duration >= lift_wait_duration_) {
    motion_state_ = MotionState::COMPLETE;
    lifting_done_ = true;
    RCLCPP_INFO(this->get_logger(), "Lifting complete; mission complete");
  }
}

double AttachServer::limit(double value,
                                    double maximum_absolute_value) const {
  return std::max(-maximum_absolute_value,
                  std::min(value, maximum_absolute_value));
}

void AttachServer::publish_cart_frame(
    const sensor_msgs::msg::LaserScan &scan) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = scan.header.stamp;
  transform.header.frame_id = scan.header.frame_id;
  transform.child_frame_id = "cart_frame";

  transform.transform.translation.x = cart_x_;
  transform.transform.translation.y = cart_y_;
  transform.transform.translation.z = 0.0;

  // cart_frame keeps the same orientation as the laser frame. For this
  // exercise, only the position between the two legs is needed.
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;

  tf_broadcaster_->sendTransform(transform);
}

void AttachServer::handle_request(
    const std::shared_ptr<GoToLoading::Request> request,
    std::shared_ptr<GoToLoading::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Request received: attach_to_shelf=%s",
              request->attach_to_shelf ? "true" : "false");

  if (!request->attach_to_shelf) {
    RCLCPP_WARN(this->get_logger(),
                "Final approach rejected: attach_to_shelf is false");
    response->complete = false;
    return;
  }

  if (!scan_received_) {
    RCLCPP_WARN(this->get_logger(),
                "Final approach rejected: no /scan message received yet");
    response->complete = false;
    return;
  }

  if (!legs_detected_) {
    RCLCPP_WARN(this->get_logger(),
                "Final approach rejected: fewer than two shelf legs "
                "detected");
    response->complete = false;
    return;
  }

  approach_requested_ = true;

  RCLCPP_INFO(this->get_logger(),
              "cart_frame publication enabled; starting approach motion");

  lifting_done_ = false;
  lift_command_sent_ = false;
  motion_state_ = MotionState::APPROACHING_CART;

  // A second executor thread (component_container_mt) keeps running the
  // control timer while this service callback checks the completion flag.
  const auto mission_start = std::chrono::steady_clock::now();
  while (rclcpp::ok() && !lifting_done_ &&
         (std::chrono::steady_clock::now() - mission_start) < mission_timeout_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  response->complete = lifting_done_;

  if (lifting_done_) {
    RCLCPP_INFO(this->get_logger(),
                "Final approach service completed successfully");
    return;
  }

  motion_state_ = MotionState::IDLE;
  approach_requested_ = false;
  velocity_publisher_->publish(geometry_msgs::msg::Twist());
  RCLCPP_ERROR(this->get_logger(), "Final approach timed out after %ld seconds",
               static_cast<long>(mission_timeout_.count()));
}

} // namespace my_components

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachServer)
