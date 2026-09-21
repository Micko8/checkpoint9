#include <chrono>
#include <memory>
#include <string>

#include "custom_interface/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"

using GoToLoading = custom_interface::srv::GoToLoading;
using namespace std::chrono_literals;

class ApproachServiceClient : public rclcpp::Node {
public:
  ApproachServiceClient() : Node("approach_service_client") {
    this->declare_parameter<bool>("final_approach", false);
    final_approach_ = this->get_parameter("final_approach").as_bool();

    client_ = this->create_client<GoToLoading>("/approach_shelf");

    RCLCPP_INFO(this->get_logger(),
                "Client ready: final_approach is %s",
                final_approach_ ? "true" : "false");
  }

  bool call_approach_service() {
    if (!final_approach_) {
      RCLCPP_INFO(this->get_logger(),
                  "Final approach disabled: /approach_shelf will not be "
                  "called");
      return true;
    }

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for /approach_shelf");
        return false;
      }

      RCLCPP_INFO(this->get_logger(),
                  "Service /approach_shelf is not available, waiting...");
    }

    auto request = std::make_shared<GoToLoading::Request>();
    request->attach_to_shelf = true;

    RCLCPP_INFO(this->get_logger(),
                "Calling /approach_shelf with attach_to_shelf=%s",
                request->attach_to_shelf ? "true" : "false");

    auto future = client_->async_send_request(request);
    const auto result = rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), future);

    if (result != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(),
                   "The call to /approach_shelf failed");
      return false;
    }

    const auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Final approach complete: %s",
                response->complete ? "true" : "false");
    return response->complete;
  }

private:
  bool final_approach_{false};
  rclcpp::Client<GoToLoading>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto client = std::make_shared<ApproachServiceClient>();
  const bool complete = client->call_approach_service();

  rclcpp::shutdown();
  return complete ? 0 : 1;
}
