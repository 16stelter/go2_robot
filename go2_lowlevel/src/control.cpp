#include "go2_lowlevel/control.hpp"

namespace go2_lowlevel {

Control::Control(rclcpp::Node::SharedPtr node, const std::string &ns, std::vector<rclcpp::Parameter> parameters)
    : node_(node) {

    qos_.best_effort();
    qos_.durability_volatile();

    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(ns + "/cmd_vel", qos_, std::bind(&Control::cmd_vel_callback, this, std::placeholders::_1));
    request_pub_ = node_->create_publisher<unitree_api::msg::Request>(ns + "/api/sport/request", qos_);

    RCLCPP_INFO(node_->get_logger(), "Control node initialized");
}

void Control::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    unitree_api::msg::Request request_msg;
    request_msg.header.identity.api_id = 1008;
    request_msg.parameter = "{\"x\":" + std::to_string(msg->linear.x) + 
                            ",\"y\":" + std::to_string(msg->linear.y) + 
                            ",\"z\":" + std::to_string(msg->angular.z) + "}";
    request_pub_->publish(request_msg);
}
}

int main(int argc, char **argv) 
{
  // init node
  rclcpp::init(argc, argv);

  // Create ros node
  auto node = std::make_shared<rclcpp::Node>("control");

  // Create cpp node
  [[maybe_unused]] go2_lowlevel::Control control(node);

  // Create executor
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  // Spin executor to process callbacks
  exec.spin();
  rclcpp::shutdown();
}
