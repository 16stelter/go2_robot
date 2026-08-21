#ifndef LOWLEVEL_LOWLEVEL_HPP_
#define LOWLEVEL_LOWLEVEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include "unitree_go/msg/low_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace go2_lowlevel {
class Lowlevel {
    public:
        explicit Lowlevel(rclcpp::Node::SharedPtr node, const std::string &ns = "",
                 std::vector<rclcpp::Parameter> parameters = {});
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::QoS qos_{rclcpp::KeepLast(1)};

        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        void lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg);
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};
}  // namespace go2_lowlevel
#endif  // LOWLEVEL_LOWLEVEL_HPP_