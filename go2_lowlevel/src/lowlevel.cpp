#include "go2_lowlevel/lowlevel.hpp"

namespace go2_lowlevel {
    
Lowlevel::Lowlevel(rclcpp::Node::SharedPtr node, const std::string &ns, std::vector<rclcpp::Parameter> parameters)
    : node_(node) {
    
    qos_.best_effort();
    qos_.durability_volatile();
    
    lowstate_sub_ = node_->create_subscription<unitree_go::msg::LowState>(ns + "/lowstate", qos_, std::bind(&Lowlevel::lowstate_callback, this, std::placeholders::_1));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(ns + "/utlidar/robot_odom", qos_, std::bind(&Lowlevel::odom_callback, this, std::placeholders::_1));
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(ns + "/joint_states", qos_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}

void Lowlevel::lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg) {
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = node_->now();
    joint_state_msg.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
                            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
                            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
    joint_state_msg.position = {msg->motor_state[0].q, msg->motor_state[1].q, msg->motor_state[2].q,
                                msg->motor_state[3].q, msg->motor_state[4].q, msg->motor_state[5].q,
                                msg->motor_state[6].q, msg->motor_state[7].q, msg->motor_state[8].q,
                                msg->motor_state[9].q, msg->motor_state[10].q, msg->motor_state[11].q};
    joint_state_msg.velocity = {msg->motor_state[0].dq, msg->motor_state[1].dq, msg->motor_state[2].dq,
                                msg->motor_state[3].dq, msg->motor_state[4].dq, msg->motor_state[5].dq,
                                msg->motor_state[6].dq, msg->motor_state[7].dq, msg->motor_state[8].dq,
                                msg->motor_state[9].dq, msg->motor_state[10].dq, msg->motor_state[11].dq};
    joint_state_pub_->publish(joint_state_msg);
}

void Lowlevel::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = node_->now();
    transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "base_link";
    transform_stamped.transform.translation.x = msg->pose.pose.position.x;
    transform_stamped.transform.translation.y = msg->pose.pose.position.y;
    transform_stamped.transform.translation.z = msg->pose.pose.position.z;
    transform_stamped.transform.rotation.x = msg->pose.pose.orientation.x;
    transform_stamped.transform.rotation.y = msg->pose.pose.orientation.y;
    transform_stamped.transform.rotation.z = msg->pose.pose.orientation.z;
    transform_stamped.transform.rotation.w = msg->pose.pose.orientation.w;
    tf_broadcaster_->sendTransform(transform_stamped);
}

}  // namespace go2_lowlevel

int main(int argc, char **argv) 
{
  // init node
  rclcpp::init(argc, argv);

  // Create ros node
  auto node = std::make_shared<rclcpp::Node>("lowlevel");

  // Create cpp node
  [[maybe_unused]] go2_lowlevel::Lowlevel lowlevel(node);

  // Create executor
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  // Spin executor to process callbacks
  exec.spin();
  rclcpp::shutdown();
}
