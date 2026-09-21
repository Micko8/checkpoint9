#include "custom_interface/srv/go_to_loading.hpp"
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class ApproachService : public rclcpp::Node {
public:
  ApproachService() : Node("approach_shelf") {
    // Create the Service Client object
    // This defines the name ('/text_recognition_service') and type (Trigger) of
    // the Service Server to connect to.
    std::string name_service = "/approach_shelf";
    client_ = this->create_client<std_srvs::srv::Trigger>(name_service);

    // Wait for the service to be available (checks every second)
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service %s not available, waiting again...",
                  name_service.c_str());
    }
  }

  void send_request() {
    // Create an empty Trigger request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Send the request asynchronously
    auto result_future = client_->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = result_future.get();
      // Log the service response
      RCLCPP_INFO(this->get_logger(), "Success: %s, Detected text: %s",
                  response->success ? "true" : "false",
                  response->message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
    }
  }

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

int main(int argc, char **argv) {
  // Initialize the ROS communication
  rclcpp::init(argc, argv);

  // Declare the node constructor
  auto client = std::make_shared<ApproachService>();

  // Run the send_request() method
  client->send_request();

  // Shutdown the ROS communication
  rclcpp::shutdown();
  return 0;
}