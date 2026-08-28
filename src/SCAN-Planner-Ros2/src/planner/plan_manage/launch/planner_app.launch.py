"""Persistent mobile terminal for starting the physical SCAN-Planner pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav2_web_share = get_package_share_directory("nav2_web")
    bridge_launch = os.path.join(
        nav2_web_share, "launch", "scanplanner_3d.launch.py"
    )
    arguments = [
        DeclareLaunchArgument("http_port", default_value="8081"),
        DeclareLaunchArgument("ws_port", default_value="8891"),
        DeclareLaunchArgument("point_limit", default_value="40000"),
        DeclareLaunchArgument("cloud_rate", default_value="5.0"),
        DeclareLaunchArgument("fixed_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="trunk"),
        DeclareLaunchArgument(
            "keypoints_file",
            default_value="",
            description="Mode 2 ROS parameter YAML containing fsm.waypoints",
        ),
        DeclareLaunchArgument(
            "reference_path_file",
            default_value="",
            description="Optional mode 3 ROS parameter YAML for the path publisher",
        ),
        DeclareLaunchArgument(
            "map_file",
            default_value=os.path.join(
                os.path.expanduser("~"), "scanplanner_maps", "scanplanner_map.pcd"),
        ),
        DeclareLaunchArgument(
            "tomogram_file",
            default_value=os.path.join(
                os.path.expanduser("~"), "scanplanner_maps", "scanplanner_map.pickle"),
        ),
        DeclareLaunchArgument(
            "map_directory",
            default_value=os.path.join(
                os.path.expanduser("~"), "scanplanner_maps"),
            description="Safe directory listed by the App map library",
        ),
        DeclareLaunchArgument(
            "fastlio_map_save_service", default_value="/map_save"
        ),
    ]
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch),
        launch_arguments={
            "http_port": LaunchConfiguration("http_port"),
            "ws_port": LaunchConfiguration("ws_port"),
            "point_limit": LaunchConfiguration("point_limit"),
            "cloud_rate": LaunchConfiguration("cloud_rate"),
            "fixed_frame": LaunchConfiguration("fixed_frame"),
            "base_frame": LaunchConfiguration("base_frame"),
            # The raw registered cloud works in both workflows: it is already
            # in map for fresh mapping and in odom for prior-map localization;
            # the 3-D client receives the TF needed to place either one.
            "odom_frame": "odom",
            "registered_cloud_topic": "/cloud_registered",
            "global_cloud_topic": "/prior_map",
            "traversable_topic": "/pct/traversable",
            "scene_path_topic": "/initial_path",
            "enable_launch_control": "true",
            "launch_profile_set": "scanplanner",
            "keypoints_file": LaunchConfiguration("keypoints_file"),
            "reference_path_file": LaunchConfiguration("reference_path_file"),
            "map_file": LaunchConfiguration("map_file"),
            "tomogram_file": LaunchConfiguration("tomogram_file"),
            "map_directory": LaunchConfiguration("map_directory"),
            "fastlio_map_save_service": LaunchConfiguration(
                "fastlio_map_save_service"
            ),
        }.items(),
    )
    return LaunchDescription(arguments + [bridge])
