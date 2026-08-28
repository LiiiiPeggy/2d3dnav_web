#!/usr/bin/env python3

"""YDLidar + RF2O odometry prior + Cartographer 2D + Web mapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_share = get_package_share_directory(
        'dog_cartographer_nav2_bringup')

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            package_share, 'launch', 'cartographer_mapping.launch.py')),
        launch_arguments={
            'start_rf2o_prior': 'True',
            'cartographer_configuration_basename':
                'ydlidar_cartographer_2d_rf2o.lua',
            'rf2o_odom_topic': '/odom_rf2o',
            'rf2o_frequency': '10.0',
        }.items(),
    )

    return LaunchDescription([mapping_launch])
