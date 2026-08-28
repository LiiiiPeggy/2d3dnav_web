#!/usr/bin/env python3

"""Frozen Cartographer graph localization + Nav2 MPPI for the real dog."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    configuration_directory = os.path.join(package_share, 'config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    pbstream = LaunchConfiguration('pbstream')
    imu_topic = LaunchConfiguration('imu_topic')
    publish_imu_tf = LaunchConfiguration('publish_imu_tf')
    start_odom_bridge = LaunchConfiguration('start_odom_bridge')
    cartographer_configuration_basename = LaunchConfiguration(
        'cartographer_configuration_basename')

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('start_lidar', default_value='True'),
        DeclareLaunchArgument(
            'start_web', default_value='False',
            description=(
                'Compatibility option. Prefer the independent '
                'nav2_web_persistent.launch.py so localization restarts do '
                'not interrupt phone connections.')),
        DeclareLaunchArgument('publish_laser_tf', default_value='True'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(maps_directory, 'dog_map.yaml'),
            description=(
                'Nav2 occupancy map; defaults to maps/dog_map.yaml in the '
                'current workspace')),
        DeclareLaunchArgument(
            'pbstream',
            default_value=os.path.join(maps_directory, 'dog_map.pbstream'),
            description=(
                'Frozen Cartographer graph; defaults to '
                'maps/dog_map.pbstream in the current workspace')),
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
        DeclareLaunchArgument(
            'imu_topic', default_value='/Devices/Imu/Data'),
        DeclareLaunchArgument(
            'start_odom_bridge', default_value='True',
            description=(
                'Publish /odom messages from Cartographer odom->base TF; '
                'the bridge never publishes TF')),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('odom_publish_rate', default_value='50.0'),
        DeclareLaunchArgument(
            'odom_use_imu_angular_velocity', default_value='False',
            description=(
                'Use the IMU gyro for Odometry angular velocity; enable '
                'this only for the Cartographer IMU profile')),
        DeclareLaunchArgument(
            'publish_imu_tf', default_value='False',
            description=(
                'Publish base_footprint->YIS320 only for the IMU profile; '
                'keep False when another robot node owns this transform')),
        DeclareLaunchArgument('imu_x', default_value='-0.14'),
        DeclareLaunchArgument('imu_y', default_value='0.0'),
        DeclareLaunchArgument('imu_z', default_value='0.31'),
        DeclareLaunchArgument('imu_roll', default_value='0.0'),
        DeclareLaunchArgument('imu_pitch', default_value='0.0'),
        DeclareLaunchArgument('imu_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'cartographer_configuration_basename',
            default_value='ydlidar_cartographer_2d_localization.lua'),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.35'),
        DeclareLaunchArgument('laser_roll', default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument(
            'laser_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument('http_port', default_value='8081'),
        DeclareLaunchArgument('ws_port', default_value='8891'),
    ]

    imu_tf = Node(
        condition=IfCondition(publish_imu_tf),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_yis320_tf',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('imu_x'),
            '--y', LaunchConfiguration('imu_y'),
            '--z', LaunchConfiguration('imu_z'),
            '--roll', LaunchConfiguration('imu_roll'),
            '--pitch', LaunchConfiguration('imu_pitch'),
            '--yaw', LaunchConfiguration('imu_yaw'),
            '--frame-id', 'base_footprint',
            '--child-frame-id', 'YIS320',
        ],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_share, 'launch', 'nav2_mppi_navigation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_lidar': LaunchConfiguration('start_lidar'),
            'start_amcl': 'False',
            'start_rf2o': 'False',
            'start_web': LaunchConfiguration('start_web'),
            'publish_laser_tf': LaunchConfiguration('publish_laser_tf'),
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'lidar_params_file': LaunchConfiguration('lidar_params_file'),
            'serial_port': LaunchConfiguration('serial_port'),
            'fixed_resolution': LaunchConfiguration('fixed_resolution'),
            'lidar_reversion': LaunchConfiguration('lidar_reversion'),
            'lidar_inverted': LaunchConfiguration('lidar_inverted'),
            'lidar_intensity': LaunchConfiguration('lidar_intensity'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'laser_x': LaunchConfiguration('laser_x'),
            'laser_y': LaunchConfiguration('laser_y'),
            'laser_z': LaunchConfiguration('laser_z'),
            'laser_roll': LaunchConfiguration('laser_roll'),
            'laser_pitch': LaunchConfiguration('laser_pitch'),
            'laser_yaw': LaunchConfiguration('laser_yaw'),
            'http_port': LaunchConfiguration('http_port'),
            'ws_port': LaunchConfiguration('ws_port'),
            'localization_backend': 'cartographer',
        }.items(),
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
            '-load_state_filename', pbstream,
            '-load_frozen_state=true',
            '-start_trajectory_with_default_topics=false',
        ],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic')),
            ('imu', imu_topic),
        ],
    )

    initial_pose_bridge = Node(
        package='dog_cartographer_nav2_bringup',
        executable='cartographer_initial_pose_bridge.py',
        name='cartographer_initial_pose_bridge',
        output='screen',
        parameters=[{
            'configuration_directory': configuration_directory,
            'configuration_basename': cartographer_configuration_basename,
        }],
    )

    odom_bridge = Node(
        condition=IfCondition(start_odom_bridge),
        package='dog_cartographer_nav2_bringup',
        executable='cartographer_tf_to_odom.py',
        name='cartographer_tf_to_odom',
        output='screen',
        parameters=[{
            'parent_frame': 'odom',
            'child_frame': 'base_footprint',
            'output_topic': LaunchConfiguration('odom_topic'),
            'publish_rate': ParameterValue(
                LaunchConfiguration('odom_publish_rate'), value_type=float),
            'use_imu_angular_velocity': ParameterValue(
                LaunchConfiguration('odom_use_imu_angular_velocity'),
                value_type=bool),
            'imu_topic': imu_topic,
        }],
    )

    return LaunchDescription(arguments + [
        LogInfo(msg=(
            'Cartographer graph localization: frozen pbstream with the '
            'selected sensor configuration; AMCL and RF2O are disabled.')),
        LogInfo(msg=['Map YAML: ', LaunchConfiguration('map')]),
        LogInfo(msg=['Cartographer state: ', LaunchConfiguration('pbstream')]),
        imu_tf,
        nav2,
        cartographer,
        odom_bridge,
        initial_pose_bridge,
    ])
