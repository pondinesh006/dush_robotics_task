#include <cstdio>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "task_dock/srv/go_to_dock.hpp"

using namespace std::chrono_literals;

class RobotClient : public rclcpp::Node
{
public:
  RobotClient()
  : Node("robot_client_node")
  {
    client_ = this->create_client<task_dock::srv::GoToDock>(
      "DockingStation"
    );

    printf("[client] Requesting charging station trip via SERVICE…\n");

    auto request = std::make_shared<task_dock::srv::GoToDock::Request>();

    if(!client_->wait_for_service(2s)){
      RCLCPP_ERROR(this->get_logger(), "Service not available..");
      return;
    }

    auto future = client_->async_send_request(request);

    printf("[client] Waiting for response (5s timeout)…\n");

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s) == rclcpp::FutureReturnCode::SUCCESS){
      printf("[client] Successful — Robot Reached the Docking station.\n");
    } else {
      printf("[client] ✘ TIMED OUT — service did not respond within 5 seconds!\n");
    }
  }

private:

  rclcpp::Client<task_dock::srv::GoToDock>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotClient>());
    rclcpp::shutdown();
    return 0;
}
