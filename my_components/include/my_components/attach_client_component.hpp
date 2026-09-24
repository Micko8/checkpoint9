#ifndef COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_
#define COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_

#include "my_components/visibility_control.h"

#include "custom_interface/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_components {

class AttachClient : public rclcpp::Node {
public:
  using GoToLoading = custom_interface::srv::GoToLoading;

  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions &options);

private:
  void timer_callback();
  void response_callback(rclcpp::Client<GoToLoading>::SharedFuture future);

  bool request_sent_{false};
  bool waiting_reported_{false};

  rclcpp::Client<GoToLoading>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace my_components

#endif // COMPOSITION__ATTACHCLIENT_COMPONENT_HPP_
