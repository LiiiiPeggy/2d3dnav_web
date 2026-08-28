#!/usr/bin/env python3

"""Long-lived Nav2 Web bridge, independent of mapping/localization launches."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory(
        'dog_cartographer_nav2_bringup')
    workspace_root = os.path.abspath(os.path.join(
        package_share, '..', '..', '..', '..'))

    arguments = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='False',
            description='Use the system clock on the real robot'),
        DeclareLaunchArgument('http_port', default_value='8081'),
        DeclareLaunchArgument('ws_port', default_value='8891'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument(
            'map_save_directory',
            default_value=os.path.join(workspace_root, 'maps')),
        DeclareLaunchArgument(
            'telemetry_rate', default_value='10.0',
            description='Web telemetry broadcasts per second'),
        DeclareLaunchArgument(
            'enable_launch_control', default_value='True',
            description=(
                'Allow the Web UI to start and stop the fixed robot launch '
                'allowlist and stream its logs')),
    ]

    bridge = Node(
        package='nav2_web',
        executable='nav2_web_bridge',
        name='nav2_web_bridge',
        output='screen',
        parameters=[{
            'http_port': ParameterValue(
                LaunchConfiguration('http_port'), value_type=int),
            'ws_port': ParameterValue(
                LaunchConfiguration('ws_port'), value_type=int),
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool),
            'telemetry_rate': ParameterValue(
                LaunchConfiguration('telemetry_rate'), value_type=float),
            'launch_control_enabled': ParameterValue(
                LaunchConfiguration('enable_launch_control'), value_type=bool),
            'map_topic': '/map',
            'path_topic': '/plan',
            'mppi_trajectories_topic': '/trajectories',
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'particle_topic': '/particle_cloud',
            'local_costmap_topic': '/local_costmap/costmap',
            'global_costmap_topic': '/global_costmap/costmap',
            'local_costmap_node': '/local_costmap/local_costmap',
            'global_costmap_node': '/global_costmap/global_costmap',
            'initial_pose_topic': '/initialpose',
            'set_initial_pose_service': '/set_initial_pose',
            'navigate_action': '/navigate_to_pose',
            'save_map_service': '/map_saver/save_map',
            'reset_localization_service':
                '/reinitialize_global_localization',
            'nomotion_update_service': '/request_nomotion_update',
            'map_save_directory': LaunchConfiguration('map_save_directory'),
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'localization_backend': 'auto',
        }],
    )

    return LaunchDescription(arguments + [
        LogInfo(msg=(
            'Starting the persistent Nav2 Web bridge. Mapping/localization/'
            'navigation launches may now be stopped without closing the phone '
            'connection. Do not also start them with start_web:=True.')),
        bridge,
    ])
