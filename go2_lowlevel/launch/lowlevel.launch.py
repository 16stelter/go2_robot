from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('go2_description'),
                'launch',
                'robot.launch.py'
            ])
        )
    )

    camera_calibration_file_arg = DeclareLaunchArgument(
        "camera_calibration_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("go2_lowlevel"),
            "config",
            "camera_calibration.yaml",
        ]),
        description="Path to the camera calibration YAML file",
    )

    lowlevel = Node(
        package = 'go2_lowlevel',
        executable = 'lowlevel',
        name = 'go2_lowlevel',
        output = 'screen',
    )

    camera = Node(
        package = 'go2_lowlevel',
        executable = 'camera',
        name = 'camera',
        output = 'screen',
        parameters=[
        {
            'camera_calibration_file': LaunchConfiguration('camera_calibration_file')
        }
    ]
        
    )

    return LaunchDescription([
        camera_calibration_file_arg,
        robot_state_publisher,
        lowlevel,
        camera,
    ])