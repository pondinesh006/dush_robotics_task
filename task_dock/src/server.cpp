#include <cstdio>
#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "task_dock/srv/go_to_dock.hpp"

using namespace std::chrono_literals;

class DockingServer : public rclcpp::Node
{
public:
  DockingServer()
  : Node("docking_server_node")
  {
    service_ = this->create_service<task_dock::srv::GoToDock>(
      "DockingStation",
      std::bind(&DockingServer::dock_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "Docking Server is activated..");
  }

private:
  void dock_request(
    const std::shared_ptr<task_dock::srv::GoToDock::Request> request,
    std::shared_ptr<task_dock::srv::GoToDock::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Moving to Docking Station..");

    for(int time=0; time < 30; time++) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    response->status = true;
  }

  rclcpp::Service<task_dock::srv::GoToDock>::SharedPtr service_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockingServer>());
  rclcpp::shutdown();
  return 0;
}
