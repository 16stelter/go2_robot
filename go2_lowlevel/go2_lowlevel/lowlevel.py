import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Image, CameraInfo
from unitree_go.msg import LowState, Go2FrontVideoData
import cv2
import numpy as np

class Go2Lowlevel(Node):
    def __init__(self):
        super().__init__('go2_lowlevel')

        state_group = ReentrantCallbackGroup()
        video_group = ReentrantCallbackGroup()

        self.lowstate_sub = self.create_subscription(LowState, 'lowstate', self.lowstate_cb, 10, callback_group=state_group)
        self.odom_sub = self.create_subscription(Odometry, 'utlidar/robot_odom', self.odom_cb, 10, callback_group=state_group)
        self.camera_sub = self.create_subscription(Go2FrontVideoData, 'frontvideostream', self.camera_cb, 10, callback_group=video_group)

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

    def lowstate_cb(self, msg):
        joint_state = JointState()
        joint_state.name = [
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        ]
        motor_state = msg.motor_state
        joint_state.position = [
            motor_state[0].q, motor_state[1].q, motor_state[2].q,   #FR
            motor_state[3].q, motor_state[4].q, motor_state[5].q,   #FL
            motor_state[6].q, motor_state[7].q, motor_state[8].q,   #RR
            motor_state[9].q, motor_state[10].q, motor_state[11].q, #RL
        ]
        joint_state.velocity = [
            motor_state[0].dq, motor_state[1].dq, motor_state[2].dq,   #FR
            motor_state[3].dq, motor_state[4].dq, motor_state[5].dq,   #FL
            motor_state[6].dq, motor_state[7].dq, motor_state[8].dq,   #RR
            motor_state[9].dq, motor_state[10].dq, motor_state[11].dq, #RL
        ]
        joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_pub.publish(joint_state)

    def odom_cb(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)

    def camera_cb(self, msg):
        return
def main(args=None):
    rclpy.init(args=args)
    node = Go2Lowlevel()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
