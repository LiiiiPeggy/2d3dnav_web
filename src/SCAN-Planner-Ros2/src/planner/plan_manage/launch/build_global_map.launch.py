"""Build and explicitly save a global PCD with the proven FAST-LIO mapper."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# T_imu_trunk for the current dog: the Mid360S is 0.15 m forward,
# 0.05 m above trunk and its front is tilted down 20 degrees.
BODY_X_IN_IMU_M = -0.1348528860
BODY_Y_IN_IMU_M = -0.02329
BODY_Z_IN_IMU_M = -0.0541676525
BODY_PITCH_IN_IMU_RAD = -0.3490658504


def _as_bool(value):
    return value.lower() in ('1', 'true', 'yes', 'on')


def _float(context, name):
    return float(LaunchConfiguration(name).perform(context))


def _vector(context, prefix):
    return [_float(context, f'{prefix}_{axis}') for axis in ('x', 'y', 'z')]


def _setup(context):
    map_file = os.path.realpath(os.path.expanduser(
        LaunchConfiguration('map_file').perform(context)))
    if not map_file.lower().endswith('.pcd'):
        raise RuntimeError('map_file must end in .pcd')
    os.makedirs(os.path.dirname(map_file), exist_ok=True)

    actions = []
    if _as_bool(LaunchConfiguration('start_livox_driver').perform(context)):
        livox_share = get_package_share_directory('livox_ros_driver2')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                livox_share, 'launch_ROS2', 'msg_MID360s_launch.py'))))

    fastlio_share = get_package_share_directory('fast_lio')
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            fastlio_share, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'config_path': os.path.join(fastlio_share, 'config'),
            'config_file': 'mid360_scanplanner.yaml',
            'world_frame': 'map',
            'imu_frame': 'fastlio_imu',
            'odom_topic': '/fastlio/imu_odometry',
            'rviz': LaunchConfiguration('start_rviz'),
            'map_file_path': map_file,
            'map_publish': LaunchConfiguration('publish_map'),
            'pcd_save': 'true',
            'pcd_save_every_n_scans': LaunchConfiguration('save_every_n_scans'),
            'pcd_save_voxel_size': LaunchConfiguration('voxel_size'),
        }.items(),
    ))
    actions.append(Node(
        package='scan_planner',
        executable='fastlio_pose_adapter',
        name='fastlio_pose_adapter',
        output='screen',
        parameters=[{
            'body_translation_in_imu': _vector(context, 'body_in_imu'),
            'body_rpy_in_imu': _vector(context, 'body_rpy_in_imu'),
            'sensor_translation_in_imu': _vector(context, 'lidar_in_imu'),
            'sensor_rpy_in_imu': _vector(context, 'lidar_rpy_in_imu'),
            'body_frame': LaunchConfiguration('body_frame').perform(context),
            'sensor_frame': LaunchConfiguration('lidar_frame').perform(context),
            'publish_tf': True,
            'publish_sensor_tf': _as_bool(
                LaunchConfiguration('publish_lidar_tf').perform(context)),
            'estimate_velocity': True,
        }],
        remappings=[
            ('fastlio_odom', '/fastlio/imu_odometry'),
            ('body_pose', '/scan_planner/body_pose'),
            ('body_odom', '/Odometry'),
            ('sensor_pose', '/scan_planner/lidar_pose'),
        ],
    ))
    return actions


def generate_launch_description():
    default_map = os.path.join(
        os.path.expanduser('~'), 'scanplanner_maps', 'scanplanner_map.pcd')
    arguments = [
        DeclareLaunchArgument('map_file', default_value=default_map),
        DeclareLaunchArgument('start_livox_driver', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument(
            'publish_map', default_value='false',
            description='Publishing the growing /Laser_map costs bandwidth; saving does not require it'),
        DeclareLaunchArgument('save_every_n_scans', default_value='5'),
        DeclareLaunchArgument('voxel_size', default_value='0.10'),
        DeclareLaunchArgument('body_frame', default_value='trunk'),
        DeclareLaunchArgument('lidar_frame', default_value='livox_frame'),
        DeclareLaunchArgument('publish_lidar_tf', default_value='true'),
    ]
    for prefix, defaults in (
        ('body_in_imu', (BODY_X_IN_IMU_M, BODY_Y_IN_IMU_M, BODY_Z_IN_IMU_M)),
        ('body_rpy_in_imu', (0.0, BODY_PITCH_IN_IMU_RAD, 0.0)),
        ('lidar_in_imu', (-0.011, -0.02329, 0.04412)),
        ('lidar_rpy_in_imu', (0.0, 0.0, 0.0)),
    ):
        for axis, value in zip(('x', 'y', 'z'), defaults):
            arguments.append(DeclareLaunchArgument(
                f'{prefix}_{axis}', default_value=str(value)))
    return LaunchDescription(arguments + [OpaqueFunction(function=_setup)])
