# BSD 3-Clause License

# Copyright (c) 2024, Intelligent Robotics Lab
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Full path to the ROS2 parameters file to use'
    ),

    composable_nodes = []

    composable_node = ComposableNode(
        package="go2_driver",
        plugin="go2_driver::Go2Driver",
        name="go2_driver",
        namespace="",
    )
    composable_nodes.append(composable_node)

    container = ComposableNodeContainer(
        name="go2_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
        output="screen",
    )

    pointclod_to_laserscan_cmd = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        namespace="",
        output="screen",
        remappings=[
            ("cloud_in", "pointcloud"),
            ("scan", "scan"),
        ],
        parameters=[{
            'target_frame': 'hesai_lidar',
            'transform_tolerance': 0.01,
            'min_height': -0.3,
            'max_height': 3.0,
            'angle_min': -3.142, #-1.5708,  # -M_PI/2
            'angle_max': 3.142, #1.5708,  # M_PI/2
            'angle_increment': 0.003141593, #0.0087,  # M_PI/360.0
            'scan_time': 0.1, #0.3333,
            'range_min': 0.1,
            'range_max': 8.0,
            'use_inf': False,
            'inf_epsilon': 1.0
        }],
    )

    ld = LaunchDescription()
    ld.add_action(container)
    ld.add_action(pointclod_to_laserscan_cmd)

    return ld
