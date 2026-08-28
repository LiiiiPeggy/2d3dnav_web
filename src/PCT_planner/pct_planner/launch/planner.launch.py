from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tomogram_path'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('body_height', default_value='0.4'),
        DeclareLaunchArgument('body_pose_topic', default_value='/scan_planner/body_pose'),
        DeclareLaunchArgument('goal_topic', default_value='/move_base_simple/goal'),
        DeclareLaunchArgument('path_topic', default_value='/initial_path'),
        Node(
            package='pct_planner',
            executable='planner_node',
            name='pct_planner',
            output='screen',
            parameters=[{
                'tomogram_path': LaunchConfiguration('tomogram_path'),
                'map_frame': LaunchConfiguration('map_frame'),
                'body_height': ParameterValue(
                    LaunchConfiguration('body_height'), value_type=float),
            }],
            remappings=[
                ('body_pose', LaunchConfiguration('body_pose_topic')),
                ('goal', LaunchConfiguration('goal_topic')),
                ('global_path', LaunchConfiguration('path_topic')),
            ],
        ),
    ])
