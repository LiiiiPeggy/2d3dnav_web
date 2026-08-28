from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pcd_path'),
        DeclareLaunchArgument('tomogram_path'),
        DeclareLaunchArgument('scene_name', default_value='plaza'),
        DeclareLaunchArgument('resolution', default_value='0.10'),
        DeclareLaunchArgument('ground_height', default_value='0.0'),
        DeclareLaunchArgument('slice_height', default_value='0.5'),
        DeclareLaunchArgument('compute_backend', default_value='auto'),
        DeclareLaunchArgument('benchmark_repeats', default_value='1'),
        DeclareLaunchArgument('traversability_kernel_size', default_value='7'),
        DeclareLaunchArgument('minimum_clearance', default_value='0.50'),
        DeclareLaunchArgument('free_clearance', default_value='0.65'),
        DeclareLaunchArgument('maximum_slope', default_value='0.36'),
        DeclareLaunchArgument('maximum_step', default_value='0.17'),
        DeclareLaunchArgument('standable_ratio', default_value='0.20'),
        DeclareLaunchArgument('safe_margin', default_value='0.40'),
        DeclareLaunchArgument('inflation', default_value='0.20'),
        Node(
            package='tomography',
            executable='tomography_node',
            name='pointcloud_tomography',
            output='screen',
            parameters=[{
                'pcd_path': LaunchConfiguration('pcd_path'),
                'tomogram_path': LaunchConfiguration('tomogram_path'),
                'scene_name': LaunchConfiguration('scene_name'),
                'resolution': ParameterValue(
                    LaunchConfiguration('resolution'), value_type=float),
                'ground_height': ParameterValue(
                    LaunchConfiguration('ground_height'), value_type=float),
                'slice_height': ParameterValue(
                    LaunchConfiguration('slice_height'), value_type=float),
                'compute_backend': LaunchConfiguration('compute_backend'),
                'benchmark_repeats': ParameterValue(
                    LaunchConfiguration('benchmark_repeats'), value_type=int),
                'traversability.kernel_size': ParameterValue(
                    LaunchConfiguration('traversability_kernel_size'), value_type=int),
                'traversability.minimum_clearance': ParameterValue(
                    LaunchConfiguration('minimum_clearance'), value_type=float),
                'traversability.free_clearance': ParameterValue(
                    LaunchConfiguration('free_clearance'), value_type=float),
                'traversability.maximum_slope': ParameterValue(
                    LaunchConfiguration('maximum_slope'), value_type=float),
                'traversability.maximum_step': ParameterValue(
                    LaunchConfiguration('maximum_step'), value_type=float),
                'traversability.standable_ratio': ParameterValue(
                    LaunchConfiguration('standable_ratio'), value_type=float),
                'traversability.safe_margin': ParameterValue(
                    LaunchConfiguration('safe_margin'), value_type=float),
                'traversability.inflation': ParameterValue(
                    LaunchConfiguration('inflation'), value_type=float),
            }],
        ),
    ])
