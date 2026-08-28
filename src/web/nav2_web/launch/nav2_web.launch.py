from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'http_port', default_value='8081', description='Web HTTP port'),
        DeclareLaunchArgument(
            'ws_port', default_value='8891', description='WebSocket port'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use Gazebo /clock; set False on the real robot'),
        DeclareLaunchArgument(
            'map_topic', default_value='/map', description='Occupancy grid topic'),
        DeclareLaunchArgument(
            'path_topic', default_value='/plan', description='Nav2 global path topic'),
        DeclareLaunchArgument(
            'mppi_trajectories_topic', default_value='/trajectories',
            description='MPPI candidate and optimal trajectories topic'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel',
            description='Velocity topic accepted by the base'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='Laser scan topic used by mapping health'),
        DeclareLaunchArgument(
            'particle_topic', default_value='/particle_cloud',
            description='AMCL particle cloud topic'),
        DeclareLaunchArgument(
            'local_costmap_topic', default_value='/local_costmap/costmap',
            description='Local costmap occupancy grid topic'),
        DeclareLaunchArgument(
            'global_costmap_topic', default_value='/global_costmap/costmap',
            description='Global costmap occupancy grid topic'),
        DeclareLaunchArgument(
            'local_costmap_node', default_value='/local_costmap/local_costmap',
            description='Local costmap parameter node'),
        DeclareLaunchArgument(
            'global_costmap_node', default_value='/global_costmap/global_costmap',
            description='Global costmap parameter node'),
        DeclareLaunchArgument(
            'initial_pose_topic', default_value='/initialpose',
            description='Localization initial pose topic'),
        DeclareLaunchArgument(
            'set_initial_pose_service', default_value='/set_initial_pose',
            description='AMCL initial pose service'),
        DeclareLaunchArgument(
            'navigate_action', default_value='/navigate_to_pose',
            description='Nav2 NavigateToPose action name'),
        DeclareLaunchArgument(
            'save_map_service', default_value='/map_saver/save_map',
            description='Nav2 map saver service name'),
        DeclareLaunchArgument(
            'reset_localization_service',
            default_value='/reinitialize_global_localization',
            description='AMCL global localization reset service name'),
        DeclareLaunchArgument(
            'nomotion_update_service', default_value='/request_nomotion_update',
            description='AMCL no-motion update service for particle refresh'),
        DeclareLaunchArgument(
            'map_save_directory', default_value='/home/w/dog_ws/maps',
            description='Absolute directory for maps saved from the Web UI'),
        DeclareLaunchArgument(
            'map_frame', default_value='map', description='Global map frame'),
        DeclareLaunchArgument(
            'odom_frame', default_value='odom', description='Local odometry frame'),
        DeclareLaunchArgument(
            'base_frame', default_value='base_link', description='Robot base frame'),
        DeclareLaunchArgument(
            'enable_launch_control', default_value='False',
            description=(
                'Enable the fixed dog launch allowlist in the Web UI')),
        DeclareLaunchArgument(
            'scene_enabled', default_value='True',
            description='Relay bounded 3D point clouds, markers, poses and TF'),
        DeclareLaunchArgument(
            'scene_point_limit', default_value='40000',
            description='Maximum xyz points sent for each 3D cloud frame'),
        DeclareLaunchArgument(
            'scene_cloud_rate', default_value='5.0',
            description='Maximum 3D point cloud WebSocket rate in Hz'),
    ]

    bridge = Node(
        package='nav2_web',
        executable='nav2_web_bridge',
        name='nav2_web_bridge',
        output='screen',
        parameters=[{
            'http_port': LaunchConfiguration('http_port'),
            'ws_port': LaunchConfiguration('ws_port'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_topic': LaunchConfiguration('map_topic'),
            'path_topic': LaunchConfiguration('path_topic'),
            'mppi_trajectories_topic': LaunchConfiguration(
                'mppi_trajectories_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'particle_topic': LaunchConfiguration('particle_topic'),
            'local_costmap_topic': LaunchConfiguration('local_costmap_topic'),
            'global_costmap_topic': LaunchConfiguration('global_costmap_topic'),
            'local_costmap_node': LaunchConfiguration('local_costmap_node'),
            'global_costmap_node': LaunchConfiguration('global_costmap_node'),
            'initial_pose_topic': LaunchConfiguration('initial_pose_topic'),
            'set_initial_pose_service': LaunchConfiguration(
                'set_initial_pose_service'),
            'navigate_action': LaunchConfiguration('navigate_action'),
            'save_map_service': LaunchConfiguration('save_map_service'),
            'reset_localization_service': LaunchConfiguration(
                'reset_localization_service'),
            'nomotion_update_service': LaunchConfiguration(
                'nomotion_update_service'),
            'map_save_directory': LaunchConfiguration('map_save_directory'),
            'map_frame': LaunchConfiguration('map_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'launch_control_enabled': ParameterValue(
                LaunchConfiguration('enable_launch_control'), value_type=bool),
            'scene_enabled': ParameterValue(
                LaunchConfiguration('scene_enabled'), value_type=bool),
            'scene_point_limit': ParameterValue(
                LaunchConfiguration('scene_point_limit'), value_type=int),
            'scene_cloud_rate': ParameterValue(
                LaunchConfiguration('scene_cloud_rate'), value_type=float),
        }],
    )

    return LaunchDescription(arguments + [bridge])
