#include "my_components/attach_client_component.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <chrono>
#include <functional>
#include <memory>

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node("attach_client", options) {
  client_ = this->create_client<GoToLoading>("/approach_shelf");

  // Le composant est chargé à l'exécution (ros2 component load) : on ne bloque
  // pas le constructeur, un timer attend que le service soit disponible.
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&AttachClient::timer_callback, this));
}

void AttachClient::timer_callback() {
  if (request_sent_) {
    return;
  }

  if (!client_->service_is_ready()) {
    if (!waiting_reported_) {
      RCLCPP_INFO(this->get_logger(),
                  "Waiting for service /approach_shelf to be available...");
      waiting_reported_ = true;
    }
    return;
  }

  auto request = std::make_shared<GoToLoading::Request>();
  request->attach_to_shelf = true;

  RCLCPP_INFO(this->get_logger(), "Calling /approach_shelf");
  request_sent_ = true;
  timer_->cancel();

  client_->async_send_request(
      request, std::bind(&AttachClient::response_callback, this,
                         std::placeholders::_1));
}

void AttachClient::response_callback(
    rclcpp::Client<GoToLoading>::SharedFuture future) {
  const auto response = future.get();

  if (response->complete) {
    RCLCPP_INFO(this->get_logger(),
                "Final approach and lifting completed successfully");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Final approach failed");
  }
}

} // namespace my_components

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)
