#!/usr/bin/env python3

"""Known-map localization + Nav2 MPPI bringup for the real YDLidar dog."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory(
        'dog_cartographer_nav2_bringup')
    ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')
    workspace_root = os.path.abspath(os.path.join(
        package_share, '..', '..', '..', '..'))
    maps_directory = os.path.join(workspace_root, 'maps')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar')
    start_amcl = LaunchConfiguration('start_amcl')
    start_rf2o = LaunchConfiguration('start_rf2o')
    start_web = LaunchConfiguration('start_web')
    publish_laser_tf = LaunchConfiguration('publish_laser_tf')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    scan_topic = LaunchConfiguration('scan_topic')

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('start_lidar', default_value='True'),
        DeclareLaunchArgument(
            'start_amcl', default_value='True',
            description=(
                'Start AMCL. The Cartographer localization launch sets this '
                'to False so only one node publishes map->odom.')),
        DeclareLaunchArgument(
            'start_rf2o', default_value='True',
            description=(
                'Generate odom->base_footprint from laser. Set False once the '
                'dog publishes a reliable real odometry TF and /odom topic.')),
        DeclareLaunchArgument(
            'start_web', default_value='False',
            description=(
                'Compatibility option. Prefer the independent '
                'nav2_web_persistent.launch.py so navigation restarts do not '
                'interrupt phone connections.')),
        DeclareLaunchArgument(
            'publish_laser_tf', default_value='True',
            description=(
                'Set False if robot_state_publisher already publishes '
                'base_footprint->laser_frame')),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(maps_directory, 'dog_map.yaml'),
            description=(
                'Nav2 occupancy map; defaults to maps/dog_map.yaml in the '
                'current workspace')),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                package_share, 'config', 'nav2_mppi_real.yaml')),
        DeclareLaunchArgument(
            'lidar_params_file',
            default_value=os.path.join(
                ydlidar_share, 'params', 'ydlidar.yaml')),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('fixed_resolution', default_value='False'),
        DeclareLaunchArgument('lidar_reversion', default_value='False'),
        DeclareLaunchArgument('lidar_inverted', default_value='False'),
        DeclareLaunchArgument('lidar_intensity', default_value='False'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.35'),
        DeclareLaunchArgument('laser_roll', default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument(
            'laser_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument('http_port', default_value='8081'),
        DeclareLaunchArgument('ws_port', default_value='8891'),
        DeclareLaunchArgument(
            'localization_backend', default_value='amcl',
            description='Localization backend reported to the Web UI'),
    ]

    laser_tf = Node(
        condition=IfCondition(publish_laser_tf),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_ydlidar_tf',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('laser_x'),
            '--y', LaunchConfiguration('laser_y'),
            '--z', LaunchConfiguration('laser_z'),
            '--roll', LaunchConfiguration('laser_roll'),
            '--pitch', LaunchConfiguration('laser_pitch'),
            '--yaw', LaunchConfiguration('laser_yaw'),
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'laser_frame',
        ],
    )

    lidar = Node(
        condition=IfCondition(start_lidar),
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('lidar_params_file'),
            {
                'port': LaunchConfiguration('serial_port'),
                'frame_id': 'laser_frame',
                'fixed_resolution': ParameterValue(
                    LaunchConfiguration('fixed_resolution'), value_type=bool),
                'reversion': ParameterValue(
                    LaunchConfiguration('lidar_reversion'), value_type=bool),
                'inverted': ParameterValue(
                    LaunchConfiguration('lidar_inverted'), value_type=bool),
                'intensity': ParameterValue(
                    LaunchConfiguration('lidar_intensity'), value_type=bool),
            },
        ],
        remappings=[('scan', scan_topic)],
    )

    rf2o = Node(
        condition=IfCondition(start_rf2o),
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'laser_scan_topic': scan_topic,
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
        }],
    )

    common_parameters = [
        params_file,
        {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
    ]
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'yaml_filename': map_yaml,
            },
        ],
        remappings=tf_remappings,
    )

    amcl = Node(
        condition=IfCondition(start_amcl),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=common_parameters,
        remappings=tf_remappings,
    )

    localization_lifecycle = Node(
        condition=IfCondition(start_amcl),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    map_only_lifecycle = Node(
        condition=UnlessCondition(start_amcl),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=common_parameters,
        remappings=tf_remappings + [('cmd_vel', 'cmd_vel_nav')],
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=common_parameters,
        remappings=tf_remappings,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=common_parameters,
        remappings=tf_remappings,
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=common_parameters,
        remappings=tf_remappings,
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=common_parameters,
        remappings=tf_remappings,
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=common_parameters,
        remappings=tf_remappings,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=common_parameters,
        remappings=tf_remappings + [
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ],
    )

    navigation_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'autostart': True,
            'node_names': [
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
    )

    web = Node(
        condition=IfCondition(start_web),
        package='nav2_web',
        executable='nav2_web_bridge',
        name='nav2_web_bridge',
        output='screen',
        parameters=[{
            'http_port': ParameterValue(
                LaunchConfiguration('http_port'), value_type=int),
            'ws_port': ParameterValue(
                LaunchConfiguration('ws_port'), value_type=int),
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'map_topic': '/map',
            'scan_topic': scan_topic,
            'path_topic': '/plan',
            'mppi_trajectories_topic': '/trajectories',
            'cmd_vel_topic': '/cmd_vel',
            'particle_topic': '/particle_cloud',
            'local_costmap_topic': '/local_costmap/costmap',
            'global_costmap_topic': '/global_costmap/costmap',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'localization_backend': LaunchConfiguration(
                'localization_backend'),
        }],
    )

    return LaunchDescription(
        arguments + [
            LogInfo(msg=(
                'Known-map Nav2 MPPI nodes are starting. The selected '
                'localization backend owns the global localization TF.')),
            laser_tf,
            lidar,
            rf2o,
            map_server,
            amcl,
            localization_lifecycle,
            map_only_lifecycle,
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            navigation_lifecycle,
            web,
        ])
