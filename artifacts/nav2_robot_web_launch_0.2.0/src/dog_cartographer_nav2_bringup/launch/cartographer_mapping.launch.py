#!/usr/bin/env python3

"""Real YDLidar + scan-only Cartographer 2D + Nav2 Web mapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
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
    configuration_directory = os.path.join(package_share, 'config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar')
    start_web = LaunchConfiguration('start_web')
    start_rf2o_prior = LaunchConfiguration('start_rf2o_prior')
    publish_laser_tf = LaunchConfiguration('publish_laser_tf')
    lidar_params_file = LaunchConfiguration('lidar_params_file')
    serial_port = LaunchConfiguration('serial_port')
    fixed_resolution = LaunchConfiguration('fixed_resolution')
    lidar_reversion = LaunchConfiguration('lidar_reversion')
    lidar_inverted = LaunchConfiguration('lidar_inverted')
    lidar_intensity = LaunchConfiguration('lidar_intensity')
    scan_topic = LaunchConfiguration('scan_topic')
    rf2o_odom_topic = LaunchConfiguration('rf2o_odom_topic')
    cartographer_configuration_basename = LaunchConfiguration(
        'cartographer_configuration_basename')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    arguments = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='False',
            description='Real hardware always uses the system clock'),
        DeclareLaunchArgument(
            'start_lidar', default_value='True',
            description='Start the real YDLidar driver'),
        DeclareLaunchArgument(
            'start_web', default_value='False',
            description=(
                'Compatibility option. Prefer the independent '
                'nav2_web_persistent.launch.py so switching mapping/'
                'navigation does not interrupt phone connections.')),
        DeclareLaunchArgument(
            'start_rf2o_prior', default_value='False',
            description=(
                'Start RF2O and feed /odom_rf2o to Cartographer as an '
                'odometry prior; RF2O does not publish TF in this mode')),
        DeclareLaunchArgument(
            'publish_laser_tf', default_value='True',
            description=(
                'Publish base_footprint->laser_frame; use False if the dog '
                'robot_state_publisher already owns this transform')),
        DeclareLaunchArgument(
            'lidar_params_file',
            default_value=os.path.join(
                ydlidar_share, 'params', 'ydlidar.yaml')),
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('fixed_resolution', default_value='False'),
        DeclareLaunchArgument('lidar_reversion', default_value='False'),
        DeclareLaunchArgument('lidar_inverted', default_value='False'),
        DeclareLaunchArgument('lidar_intensity', default_value='False'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan'),
        DeclareLaunchArgument(
            'rf2o_odom_topic', default_value='/odom_rf2o'),
        DeclareLaunchArgument(
            'rf2o_frequency', default_value='10.0'),
        DeclareLaunchArgument(
            'cartographer_configuration_basename',
            default_value='ydlidar_cartographer_2d.lua'),
        DeclareLaunchArgument(
            'laser_x', default_value='0.0'),
        DeclareLaunchArgument(
            'laser_y', default_value='0.0'),
        DeclareLaunchArgument(
            'laser_z', default_value='0.35'),
        DeclareLaunchArgument(
            'laser_roll', default_value='0.0'),
        DeclareLaunchArgument(
            'laser_pitch', default_value='0.0'),
        DeclareLaunchArgument(
            'laser_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument(
            'resolution', default_value='0.05'),
        DeclareLaunchArgument(
            'publish_period_sec', default_value='0.25'),
        DeclareLaunchArgument(
            'map_save_directory',
            default_value=os.path.join(workspace_root, 'maps')),
        DeclareLaunchArgument(
            'http_port', default_value='8081'),
        DeclareLaunchArgument(
            'ws_port', default_value='8891'),
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
            lidar_params_file,
            {
                'port': serial_port,
                'frame_id': 'laser_frame',
                'fixed_resolution': ParameterValue(
                    fixed_resolution, value_type=bool),
                'reversion': ParameterValue(
                    lidar_reversion, value_type=bool),
                'inverted': ParameterValue(
                    lidar_inverted, value_type=bool),
                'intensity': ParameterValue(
                    lidar_intensity, value_type=bool),
            },
        ],
        remappings=[('scan', scan_topic)],
    )

    # In this mapping mode RF2O is an input sensor only. Cartographer remains
    # the sole owner of map -> odom -> base_footprint, avoiding duplicate TFs.
    rf2o_prior = Node(
        condition=IfCondition(start_rf2o_prior),
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry_prior',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'laser_scan_topic': scan_topic,
            'odom_topic': rf2o_odom_topic,
            'publish_tf': False,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'rf2o_odom',
            'init_pose_from_topic': '',
            'freq': ParameterValue(
                LaunchConfiguration('rf2o_frequency'), value_type=float),
        }],
    )

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename',
            cartographer_configuration_basename,
        ],
        remappings=[
            ('scan', scan_topic),
            ('odom', rf2o_odom_topic),
        ],
    )

    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period_sec,
            '-occupancy_grid_topic', 'map',
        ],
    )

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'save_map_timeout': 10.0,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65,
            'map_subscribe_transient_local': True,
        }],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_saver',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'autostart': True,
            'node_names': ['map_saver'],
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
            'save_map_service': '/map_saver/save_map',
            'map_save_directory': LaunchConfiguration('map_save_directory'),
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
        }],
    )

    return LaunchDescription(
        arguments + [
            LogInfo(
                condition=UnlessCondition(start_rf2o_prior),
                msg=(
                    'Cartographer scan-only mapping: no IMU, no external '
                    'odom, no RF2O, no simulation.')),
            LogInfo(
                condition=IfCondition(start_rf2o_prior),
                msg=(
                    'Cartographer + RF2O mapping: RF2O publishes only the '
                    'odometry prior topic; Cartographer owns all mapping '
                    'TF.')),
            laser_tf,
            lidar,
            rf2o_prior,
            cartographer,
            occupancy_grid,
            map_saver,
            lifecycle_manager,
            web,
        ])
