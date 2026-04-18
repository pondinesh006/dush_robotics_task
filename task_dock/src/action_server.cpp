#include <cstdio>
#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "task_dock/action/go_to_dock.hpp"

using namespace std::chrono_literals;

class DockingActionServer : public rclcpp::Node
{
public:
  using GoToDock = task_dock::action::GoToDock;
  using GoalRequest = rclcpp_action::ServerGoalHandle<GoToDock>;

  DockingActionServer()
  : Node("docking_action_server")
  {
    server_ = rclcpp_action::create_server<GoToDock>(
        this,
        "DockingStation",
        std::bind(&DockingActionServer::dock_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DockingActionServer::goal_cancel, this, std::placeholders::_1),
        std::bind(&DockingActionServer::goal_accepted, this, std::placeholders::_1)
    );
  }

private:
  rclcpp_action::GoalResponse dock_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GoToDock::Goal> goal)
  {
    if (!goal->start) {
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse goal_cancel(
    const std::shared_ptr<GoalRequest>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void goal_accepted(const std::shared_ptr<GoalRequest> goal_handle) {
    std::thread(std::bind(&DockingActionServer::Execute, this, goal_handle)).detach();
  }

  void Execute(const std::shared_ptr<GoalRequest> goal_handle) {
    auto feedback = std::make_shared<GoToDock::Feedback>();
    auto result = std::make_shared<GoToDock::Result>();

    float distance = 30.0;

    for (int i=0; i<30; i++) {
        if (goal_handle->is_canceling()) {
            result->status = false;
            goal_handle->canceled(result);
            return;
        }

        distance -= 1.0;

        if (static_cast<int>(distance) % 2 == 0) {
            feedback->distance = distance;
            goal_handle->publish_feedback(feedback);
        }

        RCLCPP_INFO(this->get_logger(), "Distance remaining:: %.2f m", distance);

        std::this_thread::sleep_for(1s);
    }

    result->status = true;
    goal_handle->succeed(result);
  }

  rclcpp_action::Server<GoToDock>::SharedPtr server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockingActionServer>());
  rclcpp::shutdown();
  return 0;
}
