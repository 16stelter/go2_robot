#ifndef LOWLEVEL_CONTROL_HPP_
#define LOWLEVEL_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/request_header.hpp"
#include "unitree_api/msg/request_identity.hpp"
#include "unitree_api/msg/response.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace go2_lowlevel {
class Control {
    public:
        explicit Control(rclcpp::Node::SharedPtr node, const std::string &ns = "",
                 std::vector<rclcpp::Parameter> parameters = {});
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::QoS qos_{rclcpp::KeepLast(1)};
        
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;

        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
};
}

#endif  // LOWLEVEL_CONTROL_HPP_