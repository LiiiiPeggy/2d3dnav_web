import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    world_frame = LaunchConfiguration('world_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    odom_topic = LaunchConfiguration('odom_topic')
    map_file_path = LaunchConfiguration('map_file_path')
    map_publish = LaunchConfiguration('map_publish')
    pcd_save = LaunchConfiguration('pcd_save')
    pcd_save_every_n_scans = LaunchConfiguration('pcd_save_every_n_scans')
    pcd_save_voxel_size = LaunchConfiguration('pcd_save_voxel_size')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_world_frame_cmd = DeclareLaunchArgument(
        'world_frame', default_value='map',
        description='Fixed world frame used by odometry, registered clouds, paths, and TF'
    )
    declare_imu_frame_cmd = DeclareLaunchArgument(
        'imu_frame', default_value='body',
        description='Child frame of the FAST-LIO IMU state'
    )
    declare_odom_topic_cmd = DeclareLaunchArgument(
        'odom_topic', default_value='/Odometry',
        description='FAST-LIO raw IMU-state odometry output topic'
    )
    declare_map_file_path_cmd = DeclareLaunchArgument(
        'map_file_path', default_value='',
        description='Absolute PCD output path used by the /map_save service'
    )
    declare_map_publish_cmd = DeclareLaunchArgument(
        'map_publish', default_value='false',
        description='Publish the accumulated mapping cloud on /Laser_map'
    )
    declare_pcd_save_cmd = DeclareLaunchArgument(
        'pcd_save', default_value='false',
        description='Accumulate registered scans for an explicit /map_save request'
    )
    declare_pcd_every_cmd = DeclareLaunchArgument(
        'pcd_save_every_n_scans', default_value='5',
        description='Accumulate one scan out of every N scans'
    )
    declare_pcd_voxel_cmd = DeclareLaunchArgument(
        'pcd_save_voxel_size', default_value='0.10',
        description='Final PCD voxel size in metres; zero disables final filtering'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {
                        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                        'common.world_frame': world_frame,
                        'common.imu_frame': imu_frame,
                        'map_file_path': map_file_path,
                        'publish.map_en': ParameterValue(map_publish, value_type=bool),
                        'pcd_save.pcd_save_en': ParameterValue(pcd_save, value_type=bool),
                        'pcd_save.every_n_scans': ParameterValue(
                            pcd_save_every_n_scans, value_type=int),
                        'pcd_save.voxel_size': ParameterValue(
                            pcd_save_voxel_size, value_type=float),
                    }],
        output='screen',
        remappings=[('/Odometry', odom_topic)]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_world_frame_cmd)
    ld.add_action(declare_imu_frame_cmd)
    ld.add_action(declare_odom_topic_cmd)
    ld.add_action(declare_map_file_path_cmd)
    ld.add_action(declare_map_publish_cmd)
    ld.add_action(declare_pcd_save_cmd)
    ld.add_action(declare_pcd_every_cmd)
    ld.add_action(declare_pcd_voxel_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(rviz_node)

    return ld
