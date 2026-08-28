"""Convert a saved FAST-LIO PCD into the offline PCT tomogram."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    pcd_file = os.path.realpath(os.path.expanduser(
        LaunchConfiguration('map_file').perform(context)))
    tomogram_file = os.path.realpath(os.path.expanduser(
        LaunchConfiguration('tomogram_file').perform(context)))
    if not os.path.isfile(pcd_file):
        raise RuntimeError(f'map_file does not exist: {pcd_file}')
    if not tomogram_file.lower().endswith('.pickle'):
        raise RuntimeError('tomogram_file must end in .pickle')
    os.makedirs(os.path.dirname(tomogram_file), exist_ok=True)

    actions = [Node(
        package='tomography',
        executable='tomography_node',
        name='pointcloud_tomography',
        output='screen',
        parameters=[{
            'pcd_path': pcd_file,
            'tomogram_path': tomogram_file,
            'scene_name': LaunchConfiguration('scene_name').perform(context),
            'resolution': float(LaunchConfiguration('resolution').perform(context)),
            'ground_height': float(LaunchConfiguration('ground_height').perform(context)),
            'slice_height': float(LaunchConfiguration('slice_height').perform(context)),
            'compute_backend': LaunchConfiguration(
                'compute_backend').perform(context),
            'benchmark_repeats': int(
                LaunchConfiguration('benchmark_repeats').perform(context)),
            'traversability.kernel_size': int(
                LaunchConfiguration('traversability_kernel_size').perform(context)),
            'traversability.minimum_clearance': float(
                LaunchConfiguration('minimum_clearance').perform(context)),
            'traversability.free_clearance': float(
                LaunchConfiguration('free_clearance').perform(context)),
            'traversability.maximum_slope': float(
                LaunchConfiguration('maximum_slope').perform(context)),
            'traversability.maximum_step': float(
                LaunchConfiguration('maximum_step').perform(context)),
            'traversability.standable_ratio': float(
                LaunchConfiguration('standable_ratio').perform(context)),
            'traversability.safe_margin': float(
                LaunchConfiguration('safe_margin').perform(context)),
            'traversability.inflation': float(
                LaunchConfiguration('inflation').perform(context)),
        }],
    )]
    if LaunchConfiguration('start_rviz').perform(context).lower() in (
            '1', 'true', 'yes', 'on'):
        tomography_share = get_package_share_directory('tomography')
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                tomography_share, 'rviz', 'pct_ros2.rviz')],
            parameters=[{'use_sim_time': False}],
        ))
    return actions


def generate_launch_description():
    map_root = os.path.join(os.path.expanduser('~'), 'scanplanner_maps')
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file', default_value=os.path.join(map_root, 'scanplanner_map.pcd')),
        DeclareLaunchArgument(
            'tomogram_file', default_value=os.path.join(map_root, 'scanplanner_map.pickle')),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('scene_name', default_value='plaza'),
        DeclareLaunchArgument('resolution', default_value='0.10'),
        DeclareLaunchArgument(
            'ground_height', default_value='0.0',
            description='Map-frame ground z at the lowest traversable floor'),
        DeclareLaunchArgument('slice_height', default_value='0.5'),
        DeclareLaunchArgument(
            'compute_backend', default_value='auto',
            description='auto prefers CUDA and falls back to offline CPU'),
        DeclareLaunchArgument('benchmark_repeats', default_value='1'),
        DeclareLaunchArgument('traversability_kernel_size', default_value='7'),
        DeclareLaunchArgument('minimum_clearance', default_value='0.50'),
        DeclareLaunchArgument('free_clearance', default_value='0.65'),
        DeclareLaunchArgument('maximum_slope', default_value='0.36'),
        DeclareLaunchArgument('maximum_step', default_value='0.17'),
        DeclareLaunchArgument('standable_ratio', default_value='0.20'),
        DeclareLaunchArgument('safe_margin', default_value='0.40'),
        DeclareLaunchArgument('inflation', default_value='0.20'),
        OpaqueFunction(function=_setup),
    ])
