#include <cstdio>
#include <memory>
#include <chrono>

#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
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

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        goal_pose = true;

        RCLCPP_INFO(this->get_logger(),
            "Received Goal: x=%.2f y=%.2f yaw=%.2f",
            msg->pose.position.x,
            msg->pose.position.y,
            yaw);
    }

    void publishTransforms()
    {
        if (goal_pose) {
            geometry_msgs::msg::TransformStamped odom_to_base;

            try {
                odom_to_base = tf_buffer_->lookupTransform(
                    "odom",
                    "base_link",
                    tf2::TimePointZero
                );
            } catch (tf2::TransformException &error) {
                RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", error.what());
                return;
            }

            odom_x = odom_to_base.transform.translation.x;
            odom_y = odom_to_base.transform.translation.y;

            tf2::Quaternion odom_q(
                odom_to_base.transform.rotation.x,
                odom_to_base.transform.rotation.y,
                odom_to_base.transform.rotation.z,
                odom_to_base.transform.rotation.w
            );

            tf2::Matrix3x3(odom_q).getRPY(odom_roll, odom_pitch, odom_yaw);

            inv_x = -(odom_x * cos(odom_yaw) + odom_y * sin(odom_yaw));
            inv_y = -(-odom_y * sin(odom_yaw) + odom_y * cos(odom_yaw));
            inv_yaw = -odom_yaw;

            map_x = x + (inv_x * cos(yaw) - inv_y * sin(yaw));
            map_y = y + (inv_x * sin(yaw) + inv_y * cos(yaw));
            map_yaw = yaw + inv_yaw;

            goal_pose = false;
        }

        auto now = this->get_clock()->now();

        geometry_msgs::msg::TransformStamped map_to_odom;

        map_to_odom.header.stamp = now;
        map_to_odom.header.frame_id = "map";
        map_to_odom.child_frame_id = "odom";

        map_to_odom.transform.translation.x = map_x;
        map_to_odom.transform.translation.y = map_y;
        map_to_odom.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, map_yaw);
        q.normalize();

        map_to_odom.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(map_to_odom);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double x, y, roll, pitch, yaw;
    double odom_x, odom_y, odom_roll, odom_pitch, odom_yaw;
    double inv_x, inv_y, inv_yaw;
    double map_x{0.0}, map_y{0.0}, map_yaw{0.0};

    bool goal_pose=false;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameManagement>());
    rclcpp::shutdown();
    return 0;
}
