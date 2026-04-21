#include <cstdio>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class FrameManagement : public rclcpp::Node
{
public:
    FrameManagement()
    : Node("tf_publisher_node")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            10,
            std::bind(&FrameManagement::goalCallback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&FrameManagement::publishTransforms, this)
        );

        RCLCPP_INFO(this->get_logger(), "TF Publisher Node Started");
    }

private:
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        x = msg->pose.position.x;
        y = msg->pose.position.y;

        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );

        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(),
            "Received Goal: x=%.2f y=%.2f",
            msg->pose.position.x,
            msg->pose.position.y);
    }

    void publishTransforms()
    {
        auto now = this->get_clock()->now();

        geometry_msgs::msg::TransformStamped map_to_odom;

        map_to_odom.header.stamp = now;
        map_to_odom.header.frame_id = "map";
        map_to_odom.child_frame_id = "odom";

        map_to_odom.transform.translation.x = x;
        map_to_odom.transform.translation.y = y;
        map_to_odom.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        q.normalize();

        map_to_odom.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(map_to_odom);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    double x, y, roll, pitch, yaw;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameManagement>());
    rclcpp::shutdown();
    return 0;
}
