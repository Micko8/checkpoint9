#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

#include "custom_interface/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using GoToLoading = custom_interface::srv::GoToLoading;

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer() : Node("approach_service_server") {
    this->declare_parameter<double>("intensity_threshold", 8000.0);
    this->declare_parameter<int>("minimum_leg_separation", 10);

    intensity_threshold_ =
        this->get_parameter("intensity_threshold").as_double();
    minimum_leg_separation_ =
        this->get_parameter("minimum_leg_separation").as_int();

    scan_subscription_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&ApproachServiceServer::scan_callback, this,
                      std::placeholders::_1));

    service_ = this->create_service<GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachServiceServer::handle_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "Service /approach_shelf is ready "
                "(intensity threshold: %.0f, minimum separation: %d samples)",
                intensity_threshold_, minimum_leg_separation_);
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
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

  bool find_leg_indices(const std::vector<float> &intensities,
                        std::size_t &first_leg_index,
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

  bool valid_range(const sensor_msgs::msg::LaserScan &scan,
                   std::size_t index) const {
    if (index >= scan.ranges.size()) {
      return false;
    }

    const float range = scan.ranges[index];
    return std::isfinite(range) && range >= scan.range_min &&
           range <= scan.range_max;
  }

  void handle_request(const std::shared_ptr<GoToLoading::Request> request,
                      std::shared_ptr<GoToLoading::Response> response) {
    RCLCPP_INFO(this->get_logger(),
                "Request received: attach_to_shelf=%s",
                request->attach_to_shelf ? "true" : "false");

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

    RCLCPP_INFO(this->get_logger(),
                "Shelf center available at x=%.3f m, y=%.3f m",
                cart_x_, cart_y_);

    if (request->attach_to_shelf) {
      RCLCPP_INFO(this->get_logger(),
                  "Requested behavior: detect shelf, publish cart_frame, "
                  "move underneath it, then lift it");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Requested behavior: detect shelf and publish cart_frame "
                  "without moving or lifting");
    }

    // Leg detection is implemented, but TF publication, motion and lifting
    // are not. Reporting mission success here would therefore be incorrect.
    response->complete = false;
  }

  rclcpp::Service<GoToLoading>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_;

  double intensity_threshold_{8000.0};
  int minimum_leg_separation_{10};
  bool scan_received_{false};
  bool legs_detected_{false};
  bool missing_intensities_reported_{false};
  double cart_x_{0.0};
  double cart_y_{0.0};
  std::size_t last_first_leg_index_{0};
  std::size_t last_second_leg_index_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachServiceServer>());
  rclcpp::shutdown();
  return 0;
}
