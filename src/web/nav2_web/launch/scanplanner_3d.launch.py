"""Mobile 3D bridge defaults for physical SCAN-Planner + FAST-LIO."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    planner_web_root = os.path.join(
        get_package_share_directory('nav2_web'), 'planner_web')
    arguments = [
        DeclareLaunchArgument('http_port', default_value='8081'),
        DeclareLaunchArgument('ws_port', default_value='8891'),
        DeclareLaunchArgument('fixed_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='trunk'),
        DeclareLaunchArgument('point_limit', default_value='40000'),
        DeclareLaunchArgument('cloud_rate', default_value='5.0'),
        DeclareLaunchArgument('enable_launch_control', default_value='false'),
        DeclareLaunchArgument('launch_profile_set', default_value='scanplanner'),
        DeclareLaunchArgument('keypoints_file', default_value=''),
        DeclareLaunchArgument('reference_path_file', default_value=''),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('tomogram_file', default_value=''),
        DeclareLaunchArgument(
            'map_directory', default_value=os.path.join(
                os.path.expanduser('~'), 'scanplanner_maps')),
        DeclareLaunchArgument(
            'fastlio_map_save_service', default_value='/map_save'),
        DeclareLaunchArgument('odom_frame', default_value='map'),
        DeclareLaunchArgument(
            'registered_cloud_topic', default_value='/cloud_registered'),
        DeclareLaunchArgument(
            'global_cloud_topic', default_value='/map_generator/global_cloud'),
        DeclareLaunchArgument(
            'traversable_topic', default_value='/pct/traversable'),
        DeclareLaunchArgument('scene_path_topic', default_value='/quad_0/path'),
    ]
    bridge = Node(
        package='nav2_web',
        executable='nav2_web_bridge',
        name='scanplanner_mobile_3d_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'http_port': ParameterValue(
                LaunchConfiguration('http_port'), value_type=int),
            'ws_port': ParameterValue(
                LaunchConfiguration('ws_port'), value_type=int),
            'web_root': planner_web_root,
            'map_frame': LaunchConfiguration('fixed_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'scanplanner_goal_topic': '/move_base_simple/goal',
            'scene_enabled': True,
            'scene_point_limit': ParameterValue(
                LaunchConfiguration('point_limit'), value_type=int),
            'scene_cloud_rate': ParameterValue(
                LaunchConfiguration('cloud_rate'), value_type=float),
            'scene_registered_cloud_topic': LaunchConfiguration(
                'registered_cloud_topic'),
            'scene_livox_imu_topic': '/livox/imu',
            'scene_global_cloud_topic': LaunchConfiguration(
                'global_cloud_topic'),
            'scene_traversable_topic': LaunchConfiguration(
                'traversable_topic'),
            'scene_occupancy_topic': '/grid_map/occupancy',
            'scene_inflated_topic': '/grid_map/occupancy_inflate',
            'scene_body_pose_topic': '/scan_planner/body_pose',
            'scene_lidar_pose_topic': '/scan_planner/lidar_pose',
            'scene_fastlio_odom_topic': '/Odometry',
            'scene_path_topic': LaunchConfiguration('scene_path_topic'),
            'scene_marker_topics': [
                '/grid_map/sliding_map_bbox',
                '/goal_point',
                '/global_list',
                '/init_list',
                '/optimal_list',
                '/a_star_list',
                '/planning/self_inflation',
            ],
            'launch_control_enabled': ParameterValue(
                LaunchConfiguration('enable_launch_control'), value_type=bool),
            'launch_profile_set': LaunchConfiguration('launch_profile_set'),
            'scanplanner_keypoints_file': LaunchConfiguration('keypoints_file'),
            'scanplanner_reference_path_file': LaunchConfiguration(
                'reference_path_file'),
            'scanplanner_map_file': LaunchConfiguration('map_file'),
            'scanplanner_tomogram_file': LaunchConfiguration('tomogram_file'),
            'map_save_directory': LaunchConfiguration('map_directory'),
            'fastlio_map_save_service': LaunchConfiguration(
                'fastlio_map_save_service'),
        }],
    )
    return LaunchDescription(arguments + [bridge])
