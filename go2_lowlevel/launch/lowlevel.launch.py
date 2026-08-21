from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

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
    )

    return LaunchDescription([
        robot_state_publisher,
        lowlevel,
        camera,
    ])