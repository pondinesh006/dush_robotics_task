#include <cstdio>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "task_dock/action/go_to_dock.hpp"

using namespace std::chrono_literals;

class RobotActionClient : public rclcpp::Node
{
public:
  using GoToDock = task_dock::action::GoToDock;
  using GoalRequest = rclcpp_action::ClientGoalHandle<GoToDock>;

  RobotActionClient()
  : Node("robot_action_client")
  {
    client_ = rclcpp_action::create_client<GoToDock>(this, "DockingStation");

    client_->wait_for_action_server();

    auto goal = GoToDock::Goal();
    goal.start = true;

    auto options = rclcpp_action::Client<GoToDock>::SendGoalOptions();

    start_time_ = std::chrono::steady_clock::now();

    options.feedback_callback =
        [this](GoalRequest::SharedPtr,
            const std::shared_ptr<const GoToDock::Feedback> feedback)
    {
        printf("[feedback] Distance remaining: %.1f m\n", feedback->distance);
    };

    options.result_callback =
        [this](const GoalRequest::WrappedResult & result)
    {
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                        end_time - start_time_).count() / 1000.0;

        printf("[client] ✔ Result: Arrived at charging station! Travel time: %.1f s\n",
               duration);
    };

    client_->async_send_goal(goal, options);

    printf("[client] Goal accepted — receiving feedback…\n");
  }

private:
  rclcpp_action::Client<GoToDock>::SharedPtr client_;
  std::chrono::steady_clock::time_point start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotActionClient>());
  rclcpp::shutdown();
  return 0;
}
