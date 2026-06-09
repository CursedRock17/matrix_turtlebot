#!/usr/bin/env python3
# Composed alternative to turtlebot4_navigation's nav2.launch.py, which
# hardcodes use_composition=False and spawns ~11 separate processes. That
# many DDS participants registering at once overloads the Fast DDS discovery
# server on the robot's Pi and starves AMCL's bond heartbeat, causing the
# localization lifecycle manager to tear everything down. Running nav2 in a
# single component container avoids the discovery storm.
#
# Note: the scan topics in nav2.config.yaml are absolute (/scan) instead of
# the SetRemap trick the turtlebot4_navigation wrapper uses, so no remaps
# are needed here. This assumes the robot publishes at the root namespace.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('turtlebot4_bringup'),
            'config',
            'nav2.config.yaml']),
        description='Nav2 parameters'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
]


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_composition': 'True',
            'container_name': 'nav2_container',
        }.items()
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(container)
    ld.add_action(navigation)
    return ld
