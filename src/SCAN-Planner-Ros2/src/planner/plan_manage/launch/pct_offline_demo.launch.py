"""Safe PCT/App demo using a prior PCD, tomogram, and fixed start pose."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _float(context, name):
    return float(LaunchConfiguration(name).perform(context))


def _setup(context):
    map_file = os.path.realpath(os.path.expanduser(
        LaunchConfiguration('map_file').perform(context)))
    tomogram_file = os.path.realpath(os.path.expanduser(
        LaunchConfiguration('tomogram_file').perform(context)))
    if not os.path.isfile(map_file):
        raise RuntimeError(f'map_file does not exist: {map_file}')
    if not os.path.isfile(tomogram_file):
        raise RuntimeError(f'tomogram_file does not exist: {tomogram_file}')

    body_frame = LaunchConfiguration('body_frame').perform(context)
    body_height = _float(context, 'body_height')
    actions = [
        Node(
            package='icp_relocalization', executable='icp_node',
            name='pct_demo_map_publisher', output='screen',
            parameters=[{
                'map_path': map_file,
                'map_frame_id': 'map',
                'map_voxel_leaf_size': _float(context, 'map_voxel_size'),
                'pcl_type': 'pointcloud',
            }],
            remappings=[
                ('prior_map', '/prior_map'),
                ('pointcloud', '/pct_demo/unused_cloud'),
            ],
        ),
        Node(
            package='scan_planner',
            executable='pct_demo_pose_publisher.py',
            name='pct_demo_pose_publisher', output='screen',
            parameters=[{
                'map_frame': 'map',
                'body_frame': body_frame,
                'x': _float(context, 'demo_start_x'),
                'y': _float(context, 'demo_start_y'),
                'ground_z': _float(context, 'demo_start_ground_z'),
                'body_height': body_height,
                'yaw': _float(context, 'demo_start_yaw'),
            }],
            remappings=[
                ('body_pose', '/scan_planner/body_pose'),
                ('initialpose', '/initialpose'),
            ],
        ),
        Node(
            package='pct_planner', executable='planner_node',
            name='pct_demo_global_planner', output='screen',
            parameters=[{
                'tomogram_path': tomogram_file,
                'map_frame': 'map',
                'body_height': body_height,
                # The App publishes the picked ground-cloud height after
                # adding body_height. Convert back to ground Z for PCT.
                'goal_z_is_body': True,
            }],
            remappings=[
                ('body_pose', '/scan_planner/body_pose'),
                ('goal', '/move_base_simple/goal'),
                ('waypoints', '/pct_waypoints'),
                ('global_path', '/initial_path'),
                ('traversable_cloud', '/pct/traversable'),
            ],
        ),
    ]
    if LaunchConfiguration('start_rviz').perform(context).lower() in (
            '1', 'true', 'yes', 'on'):
        localization_share = get_package_share_directory(
            'icp_relocalization')
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                localization_share, 'rviz', 'loam_livox.rviz')],
            parameters=[{'use_sim_time': False}],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_file'),
        DeclareLaunchArgument('tomogram_file'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('body_frame', default_value='trunk'),
        DeclareLaunchArgument('body_height', default_value='0.40'),
        DeclareLaunchArgument('demo_start_x', default_value='0.0'),
        DeclareLaunchArgument('demo_start_y', default_value='0.0'),
        DeclareLaunchArgument('demo_start_ground_z', default_value='0.0'),
        DeclareLaunchArgument('demo_start_yaw', default_value='0.0'),
        DeclareLaunchArgument('map_voxel_size', default_value='0.15'),
        OpaqueFunction(function=_setup),
    ])
