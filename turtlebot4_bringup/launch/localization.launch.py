#!/usr/bin/env python3
# Wraps turtlebot4_navigation's localization.launch.py with the localization
# lifecycle manager's bond watchdog DISABLED.
#
# On large maps (first_floor is 2496x745 ~ 1.86 MB) map_server's single-threaded
# executor gets blocked long enough — serializing/sending the latched map to new
# subscribers, under laptop CPU contention with RViz — that it misses the 4 s
# bond heartbeat. lifecycle_manager_localization then declares
# "CRITICAL FAILURE: SERVER map_server IS DOWN" and tears the whole localization
# stack down ~4 s after it activates, so AMCL is dead before the robot undocks
# (2026-06-16 field log). Small maps (robotics_lab) send fast enough to never
# trip it, which is why this only shows up on first_floor.
#
# Same fix as nav2.launch.py on the sibling lifecycle manager. SetParameter is
# required because the stock localization_launch.py passes the lifecycle manager
# only {autostart, node_names} — its params_file never reaches it, so a YAML
# bond_timeout section is silently ignored. Lifecycle transitions still run; we
# only drop the liveness watchdog (nav2's is already disabled the same way).

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import SetParameter

ARGUMENTS = [
    DeclareLaunchArgument('map',
                          description='Full path to the map yaml file to load'),
    DeclareLaunchArgument(
        'params',
        default_value=PathJoinSubstitution([
            get_package_share_directory('turtlebot4_bringup'),
            'config',
            'localization.config.yaml']),
        description='Localization (map_server + amcl) parameters'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace (no leading /), '
                                      'empty for the single-robot default'),
]


def generate_launch_description():
    pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')

    localization = GroupAction([
        # Disable the localization lifecycle manager's bond watchdog (see header).
        SetParameter('bond_timeout', 0.0),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [pkg_turtlebot4_navigation, 'launch', 'localization.launch.py'])),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'map': LaunchConfiguration('map'),
                'params': LaunchConfiguration('params'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()),
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(localization)
    return ld
