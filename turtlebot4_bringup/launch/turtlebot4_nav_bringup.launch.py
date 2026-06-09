#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml

# Find Any Necessary Packages    
pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_bringup')
pkg_turtlebot4_visualizer = get_package_share_directory('turtlebot4_viz')
pkg_turtlebot4_navigator = get_package_share_directory('turtlebot4_navigation')

# Add All Launch Arguments
ARGUMENTS = [
    DeclareLaunchArgument(
        'nav2_config',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_bringup, 'config', 'nav2.config.yaml']),
        description='Turtlebot4 Navigation 2 Param file'
    ),
    DeclareLaunchArgument(
        'localization_config',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_bringup, 'config', 'localization.config.yaml']),
        description='Turtlebot4 Localization Param file'
    ),
    DeclareLaunchArgument(
        'namespace', default_value='',
        description='Robot Namespace'
    ),
    DeclareLaunchArgument(
        'map', default_value='/home/ros/Documents/matrix_turtlebot/maps/second_half_building.yaml',
        description='Full path to map yaml file'
    )

]

def generate_launch_description():
    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot4_visualizer, 'launch', 'view_navigation.launch.py')
        )
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot4_navigator, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false'
        }.items()
    )
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot4_navigator, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('nav2_config')
        }.items()
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(viz_launch)
    ld.add_action(localization_launch)
    #ld.add_action(navigation_launch)

    return ld
