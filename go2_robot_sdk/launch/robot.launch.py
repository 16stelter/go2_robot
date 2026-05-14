import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.actions import GroupAction, OpaqueFunction
import xacro, tempfile

def spawn_nodes(context, namespace, ip, use_rviz, use_foxglove, use_teleop, use_sim_time):
    robot_ns = context.perform_substitution(namespace)
    robot_ip = context.perform_substitution(ip)

    package_dir = get_package_share_directory('go2_robot_sdk')
    description_dir = get_package_share_directory('go2_description')

    urdf = os.path.join(description_dir, 'urdf', 'unitree_go2_robot.xacro')
    rviz_config = os.path.join(package_dir, 'config', 'single_robot_conf.rviz')
    twist_mux_config = os.path.join(package_dir, 'config', 'twist_mux.yaml')

    ns_prefix = f"{robot_ns}/" if robot_ns else ""

    with open(twist_mux_config) as f:
        config = f.read()
    namespaced_config = config.replace("__robot_ns__", ns_prefix)
    tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    tmp.write(namespaced_config)
    tmp.close()
    namespaced_mux_config = tmp.name

    with open(rviz_config) as f:
        config = f.read()
    namespaced_config = config.replace("__tf_prefix__", robot_ns)
    tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    tmp.write(namespaced_config)
    tmp.close()
    namespaced_rviz_config = tmp.name    

    robot_desc = xacro.process(
        urdf,
        mappings={"robot_ns": robot_ns},
    )

    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='go2_robot_state_publisher',
        output='screen',
        namespace=robot_ns,
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
            'frame_prefix': ns_prefix
        }],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    core_nodes = [
        # Main robot driver
        # While the ros2 side accepts namespaces, the webrtc interface does not, it will always use /cmd_vel_out and /webrtc_req
        # in general, this part requires some refactoring.
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            name='go2_driver_node',
            namespace=robot_ns,
            output='screen',
            parameters=[{
                'robot_ip': robot_ip,
                'token': "",
                'conn_type': "webrtc",
                'frame_prefix': ns_prefix
            }],
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]
        ),
        # LiDAR processing node
        Node(
            package='lidar_processor',
            executable='lidar_to_pointcloud',
            name='lidar_to_pointcloud',
            namespace=robot_ns,
            parameters=[{
                'robot_ip_lst': [robot_ip],
                'map_name': "3d_map",
                'map_save': "true"
            }],
        ),
        # Advanced point cloud aggregator
        Node(
            package='lidar_processor',
            executable='pointcloud_aggregator',
            name='pointcloud_aggregator',
            namespace=robot_ns,
            parameters=[{
                'max_range': 20.0,
                'min_range': 0.1,
                'height_filter_min': -2.0,
                'height_filter_max': 3.0,
                'downsample_rate': 5,
                'publish_rate': 10.0
            }],
            remappings=[("/pointcloud/aggregated", "pointcloud/aggregated"),
                        ("/pointcloud/filtered", "pointcloud/filtered"),
                        ("/pointcloud/downsampled", "pointcloud/downsampled")],
        )
    ]

    teleop_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        namespace=robot_ns,
        condition=IfCondition(use_teleop),
        parameters=[
            {'use_sim_time': use_sim_time},
            namespaced_mux_config
        ],
        remappings=[("cmd_vel_out", "/cmd_vel_out")]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(use_rviz),
        name='go2_rviz2',
        output='screen',
        namespace=robot_ns,
        arguments=['-d', namespaced_rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    foxglove_launch = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch', 'foxglove_bridge_launch.xml'
    )

    foxglove_launch_group = GroupAction([
        PushRosNamespace(robot_ns),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(foxglove_launch),
            condition=IfCondition(use_foxglove),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])

    return [
        robot_state_node,
        *core_nodes,
        teleop_mux_node,
        rviz_node,
        foxglove_launch_group
    ]

def generate_launch_description():
    robot_ns_arg = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace"
    )

    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.122.13",
        description="IP of the robot's SDK interface"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch Rviz2"
    )

    use_foxglove_arg = DeclareLaunchArgument(
        "use_foxglove",
        default_value="true",
        description="Launch Foxglove Bridge"
    )

    use_teleop_arg = DeclareLaunchArgument(
        "use_teleop",
        default_value="true",
        description="Launch Teleoperation"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Whether to use sim time"
    )

    robot_ns = LaunchConfiguration("robot_ns")
    robot_ip = LaunchConfiguration("robot_ip")
    use_rviz = LaunchConfiguration("use_rviz")
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_teleop = LaunchConfiguration("use_teleop")
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        robot_ns_arg,
        robot_ip_arg,
        use_rviz_arg,
        use_foxglove_arg,
        use_teleop_arg,
        use_sim_time_arg,
        OpaqueFunction(function=spawn_nodes, args=[robot_ns, robot_ip, use_rviz, use_foxglove, use_teleop, use_sim_time])
    ])