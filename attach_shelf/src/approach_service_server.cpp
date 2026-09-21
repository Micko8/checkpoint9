#include <functional>
#include <memory>

#include "custom_interface/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"

using GoToLoading = custom_interface::srv::GoToLoading;

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer() : Node("approach_service_server") {
    service_ = this->create_service<GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachServiceServer::handle_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "Service /approach_shelf is ready");
  }

private:
  void handle_request(const std::shared_ptr<GoToLoading::Request> request,
                      std::shared_ptr<GoToLoading::Response> response) {
    RCLCPP_INFO(this->get_logger(),
                "Request received: attach_to_shelf=%s",
                request->attach_to_shelf ? "true" : "false");

    if (request->attach_to_shelf) {
      RCLCPP_INFO(this->get_logger(),
                  "Requested behavior: detect shelf, publish cart_frame, "
                  "move underneath it, then lift it");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Requested behavior: detect shelf and publish cart_frame "
                  "without moving or lifting");
    }

    // This first skeleton validates only the client/server communication.
    // Detection, motion and lifting are not implemented yet, so reporting
    // success here would be incorrect.
    response->complete = false;
  }

  rclcpp::Service<GoToLoading>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachServiceServer>());
  rclcpp::shutdown();
  return 0;
}
