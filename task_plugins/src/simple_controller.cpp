#include "task_plugins/simple_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include <cmath>

namespace task_plugins
{

void SimpleController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)
{
    auto node = parent.lock();
    logger_ = node->get_logger();

    node->declare_parameter(name + ".k_angular", 1.5);
    node->declare_parameter(name + ".linear_vel", 0.2);

    node->get_parameter(name + ".k_angular", k_angular_);
    node->get_parameter(name + ".linear_vel", linear_vel_);

    RCLCPP_INFO(logger_, "SimpleController Configured");
}

void SimpleController::activate()
{
    RCLCPP_INFO(logger_, "SimpleController activated");
}

void SimpleController::deactivate()
{
    RCLCPP_INFO(logger_, "SimpleController deactivated");
}

void SimpleController::cleanup()
{
    RCLCPP_INFO(logger_, "SimpleController cleaned up");
}

void SimpleController::setPlan(const nav_msgs::msg::Path & path)
{
    path_ = path;
    current_index_ = 0;
}

geometry_msgs::msg::TwistStamped
SimpleController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *)
{
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = pose.header.stamp;

    if (path_.poses.empty()) {
        return cmd;
    }

    double min_dist = std::numeric_limits<double>::max();
    size_t closest_index = current_index_;

    for (size_t i = current_index_; i < path_.poses.size(); i++) {
        double dx = path_.poses[i].pose.position.x - pose.pose.position.x;
        double dy = path_.poses[i].pose.position.y - pose.pose.position.y;
        double dist = hypot(dx, dy);

        if (dist < min_dist) {
            min_dist = dist;
            closest_index = i;
        }
    }

    current_index_ = closest_index;

    auto target = path_.poses[current_index_];

    double dx = target.pose.position.x - pose.pose.position.x;
    double dy = target.pose.position.y - pose.pose.position.y;

    double target_angle = atan2(dy, dx);
    double yaw = tf2::getYaw(pose.pose.orientation);
    double error = target_angle - yaw;

    cmd.twist.linear.x = linear_vel_;
    cmd.twist.angular.z = k_angular_ * error;

    return cmd;
}

void SimpleController::setSpeedLimit(
    const double & speed_limit,
    const bool & percentage)
{
    if (percentage) {
        linear_vel_ = linear_vel_ * speed_limit;
    } else {
        linear_vel_ = speed_limit;
    }

    RCLCPP_INFO(logger_, "Speed limit updated: %f", linear_vel_);
}

} // namespace task_plugins

PLUGINLIB_EXPORT_CLASS(task_plugins::SimpleController, nav2_core::Controller)
