#pragma once

#include "nav2_core/controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace task_plugins
{
class SimpleController : public nav2_core::Controller
{
public:
  SimpleController() = default;
  ~SimpleController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  nav_msgs::msg::Path path_;
  size_t current_index_{0};

  double k_angular_{1.5};
  double linear_vel_{0.2};

  rclcpp::Logger logger_{rclcpp::get_logger("SimpleController")};
};

} // namespace task_plugins
