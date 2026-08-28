"""Relocalize in a prior PCD, run PCT globally, and feed SCAN mode 3."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# T_imu_body for a Mid-360S mounted 0.15 m forward and 0.05 m above trunk
# (the dog IMU origin), with its x/front side tilted down 20 deg.
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


def _rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _transpose(matrix):
    return [[matrix[j][i] for j in range(3)] for i in range(3)]


def _matmul(left, right):
    return [[sum(left[i][k] * right[k][j] for k in range(3))
             for j in range(3)] for i in range(3)]


def _matvec(matrix, vector):
    return [sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3)]


def _rpy(matrix):
    pitch = math.asin(max(-1.0, min(1.0, -matrix[2][0])))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        roll = math.atan2(-matrix[1][2], matrix[1][1])
        yaw = 0.0
    return [roll, pitch, yaw]


def _relative_pose(parent_translation, parent_rpy, child_translation, child_rpy):
    imu_from_parent = _rotation(parent_rpy)
    parent_from_imu = _transpose(imu_from_parent)
    translation = _matvec(
        parent_from_imu,
        [child_translation[i] - parent_translation[i] for i in range(3)])
    rotation = _matmul(parent_from_imu, _rotation(child_rpy))
    return translation, _rpy(rotation)


def _setup(context):
    map_file = os.path.realpath(os.path.expanduser(
        LaunchConfiguration('map_file').perform(context)))
    tomogram_file = os.path.realpath(os.path.expanduser(
        LaunchConfiguration('tomogram_file').perform(context)))
    start_pct = _as_bool(LaunchConfiguration('start_pct').perform(context))
    if not os.path.isfile(map_file):
        raise RuntimeError(f'prior PCD does not exist: {map_file}')
    if start_pct and not os.path.isfile(tomogram_file):
        raise RuntimeError(f'PCT tomogram does not exist: {tomogram_file}')

    start_driver = _as_bool(LaunchConfiguration('start_livox_driver').perform(context))
    start_rviz = _as_bool(LaunchConfiguration('start_rviz').perform(context))
    start_mobile = _as_bool(LaunchConfiguration('start_mobile_3d').perform(context))
    start_localization = _as_bool(
        LaunchConfiguration('start_localization').perform(context))
    start_scanplanner = _as_bool(
        LaunchConfiguration('start_scanplanner').perform(context))
    enable_control = _as_bool(LaunchConfiguration('enable_control').perform(context))
    body_translation = _vector(context, 'body_in_imu')
    body_rpy = _vector(context, 'body_rpy_in_imu')
    lidar_translation = _vector(context, 'lidar_in_imu')
    lidar_rpy = _vector(context, 'lidar_rpy_in_imu')
    body_to_lidar_translation, body_to_lidar_rpy = _relative_pose(
        body_translation, body_rpy, lidar_translation, lidar_rpy)
    body_frame = LaunchConfiguration('body_frame').perform(context)
    lidar_frame = LaunchConfiguration('lidar_frame').perform(context)
    fastlio_imu_frame = LaunchConfiguration('fastlio_imu_frame').perform(context)
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic').perform(context)

    actions = []
    if start_driver:
        livox_share = get_package_share_directory('livox_ros_driver2')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                livox_share, 'launch_ROS2', 'msg_MID360s_launch.py'))))

    localization_share = get_package_share_directory('fast_lio_localization')
    localization_config = os.path.join(
        localization_share, 'config', 'mid360_scanplanner_localization.yaml')
    localization_actions = [
        Node(
            package='icp_relocalization',
            executable='transform_publisher',
            name='relocalization_transform_publisher',
            output='screen',
            parameters=[{
                'map_frame_id': 'map',
                'odom_frame_id': 'odom',
                'sensor_translation_in_odom': lidar_translation,
                'sensor_rpy_in_odom': lidar_rpy,
            }],
            remappings=[
                ('icp_sensor_result', '/relocalization/icp_sensor_result'),
                ('icp_result', '/icp_result'),
            ],
        ),
        Node(
            package='icp_relocalization',
            executable='icp_node',
            name='icp_relocalization',
            output='screen',
            parameters=[{
                'map_path': map_file,
                'map_frame_id': 'map',
                'initial_x': _float(context, 'initial_x'),
                'initial_y': _float(context, 'initial_y'),
                'initial_z': _float(context, 'initial_z'),
                'initial_yaw': _float(context, 'initial_yaw'),
                # RViz/App initial-pose tools select the ground surface.  ICP
                # aligns the LiDAR from the robot/base pose, so lift only
                # interactive initial-pose messages by the configured height.
                'initialpose_ground_to_robot_z': _float(context, 'body_height'),
                'map_voxel_leaf_size': _float(context, 'icp_map_voxel_size'),
                'cloud_voxel_leaf_size': _float(context, 'icp_scan_voxel_size'),
                'max_correspondence_distance': _float(
                    context, 'icp_max_correspondence_distance'),
                'fitness_score_threshold': _float(context, 'icp_fitness_threshold'),
                'required_convergences': int(
                    LaunchConfiguration('icp_required_convergences').perform(context)),
                'pcl_type': 'livox',
                'sensor_translation_in_initial_frame': body_to_lidar_translation,
                'sensor_rpy_in_initial_frame': body_to_lidar_rpy,
            }],
            remappings=[
                ('livox_cloud', '/livox/lidar'),
                ('initialpose', '/initialpose'),
                ('icp_sensor_result', '/relocalization/icp_sensor_result'),
                ('prior_map', '/prior_map'),
                ('transformed_cloud', '/relocalization/aligned_cloud'),
            ],
        ),
        Node(
            package='fast_lio_localization',
            executable='fastlio_mapping',
            name='fastlio_localization',
            output='screen',
            parameters=[
                localization_config,
                {
                    'prior_map_path': map_file,
                    'common.odom_frame_id': 'odom',
                    'common.sensor_frame_id': fastlio_imu_frame,
                    'common.base_frame_id': body_frame,
                    'common.send_odom_base_tf': False,
                    'mapping.extrinsic_T': lidar_translation,
                    'mapping.extrinsic_R': [
                        value for row in _rotation(lidar_rpy) for value in row],
                },
            ],
            remappings=[('/Odometry', '/fastlio/imu_odometry')],
        ),
        Node(
            package='scan_planner', executable='fastlio_pose_adapter',
            name='fastlio_pose_adapter', output='screen',
            parameters=[{
                'body_translation_in_imu': body_translation,
                'body_rpy_in_imu': body_rpy,
                'sensor_translation_in_imu': lidar_translation,
                'sensor_rpy_in_imu': lidar_rpy,
                'body_frame': body_frame,
                'sensor_frame': lidar_frame,
                'publish_tf': True,
                'publish_sensor_tf': _as_bool(
                    LaunchConfiguration('publish_lidar_tf').perform(context)),
                'estimate_velocity': True,
            }],
            remappings=[
                ('fastlio_odom', '/fastlio/imu_odometry'),
                ('body_pose', '/scan_planner/body_pose_local'),
                ('body_odom', '/Odometry'),
                ('sensor_pose', '/scan_planner/lidar_pose_local'),
            ],
        ),
        Node(
            package='scan_planner', executable='global_frame_adapter',
            name='scanplanner_global_frame_adapter', output='screen',
            parameters=[{'global_frame': 'map'}],
            remappings=[
                ('body_pose_local', '/scan_planner/body_pose_local'),
                ('sensor_pose_local', '/scan_planner/lidar_pose_local'),
                ('cloud_local', '/cloud_registered'),
                ('body_pose_global', '/scan_planner/body_pose'),
                ('sensor_pose_global', '/scan_planner/lidar_pose'),
                ('cloud_global', '/scan_planner/cloud_registered_map'),
            ],
        ),
        Node(
            package='scan_planner', executable='fastlio_input_monitor.py',
            name='fastlio_input_monitor', output='screen',
            parameters=[{
                'expected_frame': 'map', 'max_message_age': 0.3,
                'max_stamp_skew': 0.25, 'min_odom_hz': 5.0,
                'min_cloud_hz': 3.0,
            }],
            remappings=[
                ('body_pose', '/scan_planner/body_pose'),
                ('cloud', '/scan_planner/cloud_registered_map'),
                ('inputs_ready', '/scan_planner/fastlio_inputs_ready'),
            ],
        ),
    ]
    if start_localization:
        actions.extend(localization_actions)

    if start_pct:
        actions.append(Node(
            package='pct_planner', executable='planner_node',
            name='pct_global_planner', output='screen',
            parameters=[{
                'tomogram_path': tomogram_file,
                'map_frame': 'map',
                'body_height': _float(context, 'body_height'),
                'goal_z_is_body': True,
            }],
            remappings=[
                ('body_pose', '/scan_planner/body_pose'),
                ('goal', '/move_base_simple/goal'),
                ('waypoints', '/pct_waypoints'),
                ('global_path', '/initial_path'),
                ('traversable_cloud', '/pct/traversable'),
            ],
        ))

    scan_share = get_package_share_directory('scan_planner')
    if start_scanplanner:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                scan_share, 'launch', 'run.launch.py')),
            launch_arguments={
                'is_real_world': 'true', 'use_sim_time': 'false',
                'sensor_type': 'lidar', 'navi_mode': '3',
                'controller_mode': 'closed_loop',
                'start_controller': 'true' if enable_control else 'false',
                'require_inputs_ready': 'true', 'publish_robot_state': 'false',
                'body_pose_topic': '/scan_planner/body_pose',
                'sensor_pose_topic': '/scan_planner/lidar_pose',
                'cloud_topic': '/scan_planner/cloud_registered_map',
                'cmd_vel_topic': cmd_vel_topic,
                'world_frame': 'map', 'cloud_is_world': 'true',
                'need_extrinsic': 'false', 'reference_path_z_is_body': 'true',
            }.items(),
        ))

    if start_mobile:
        web_share = get_package_share_directory('nav2_web')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                web_share, 'launch', 'scanplanner_3d.launch.py')),
            launch_arguments={
                'http_port': LaunchConfiguration('mobile_http_port'),
                'ws_port': LaunchConfiguration('mobile_ws_port'),
                'fixed_frame': 'map', 'base_frame': body_frame,
                'odom_frame': 'odom',
                'registered_cloud_topic': '/scan_planner/cloud_registered_map',
                'global_cloud_topic': '/prior_map',
                'scene_path_topic': '/initial_path',
                'enable_launch_control': 'false',
            }.items(),
        ))
    if start_rviz:
        icp_share = get_package_share_directory('icp_relocalization')
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(
                icp_share, 'rviz', 'loam_livox.rviz')],
            parameters=[{'use_sim_time': False}],
        ))
    return actions


def generate_launch_description():
    map_root = os.path.join(os.path.expanduser('~'), 'scanplanner_maps')
    arguments = [
        DeclareLaunchArgument('map_file', default_value=os.path.join(
            map_root, 'scanplanner_map.pcd')),
        DeclareLaunchArgument('tomogram_file', default_value=os.path.join(
            map_root, 'scanplanner_map.pickle')),
        DeclareLaunchArgument('start_livox_driver', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_mobile_3d', default_value='false'),
        DeclareLaunchArgument('start_localization', default_value='true'),
        DeclareLaunchArgument('start_pct', default_value='true'),
        DeclareLaunchArgument('start_scanplanner', default_value='true'),
        DeclareLaunchArgument('enable_control', default_value='false'),
        DeclareLaunchArgument('mobile_http_port', default_value='8081'),
        DeclareLaunchArgument('mobile_ws_port', default_value='8891'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('fastlio_imu_frame', default_value='fastlio_imu'),
        DeclareLaunchArgument('body_frame', default_value='trunk'),
        DeclareLaunchArgument('lidar_frame', default_value='livox_frame'),
        DeclareLaunchArgument('publish_lidar_tf', default_value='true'),
        DeclareLaunchArgument('body_height', default_value='0.4'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_z', default_value='0.4'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('icp_map_voxel_size', default_value='0.30'),
        DeclareLaunchArgument('icp_scan_voxel_size', default_value='0.20'),
        DeclareLaunchArgument('icp_max_correspondence_distance', default_value='1.0'),
        DeclareLaunchArgument('icp_fitness_threshold', default_value='0.30'),
        DeclareLaunchArgument('icp_required_convergences', default_value='5'),
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
