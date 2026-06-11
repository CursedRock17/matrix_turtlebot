#!/usr/bin/env python3
# One-shot bringup for the whole laptop-side navigation stack: RViz +
# localization + our composed nav2 launch, replacing three of the four
# terminals in docs/navigate_to_a_goal.md. The custom navigation node
# (nav_to_node, nav_patrol_loop, ...) stays in its own terminal since it is
# the piece that gets stopped and rerun between tests.
#
# Run it from the workspace root: the default map path and the per-map
# locations files (maps/<name>.locations.yaml) resolve relative to the cwd.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument(
        'map',
        default_value='./maps/robotics_lab.yaml',
        description='Map yaml to localize on (needs a .locations.yaml beside it)'),
    DeclareLaunchArgument(
        'localization_params',
        default_value=PathJoinSubstitution([
            get_package_share_directory('turtlebot4_bringup'),
            'config',
            'localization.config.yaml']),
        description='AMCL/map_server parameters'),
    DeclareLaunchArgument(
        'nav2_params',
        default_value=PathJoinSubstitution([
            get_package_share_directory('turtlebot4_bringup'),
            'config',
            'nav2.config.yaml']),
        description='Nav2 parameters'),
    DeclareLaunchArgument('use_rviz', default_value='true',
                          choices=['true', 'false'],
                          description='Also start RViz'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace (no leading /), '
                                      'empty for the single-robot default'),
]


def generate_launch_description():
    pkg_turtlebot4_viz = get_package_share_directory('turtlebot4_viz')
    pkg_turtlebot4_navigation = get_package_share_directory('turtlebot4_navigation')
    pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_turtlebot4_viz, 'launch', 'view_navigation.launch.py'])),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
        }.items())

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_turtlebot4_navigation, 'launch', 'localization.launch.py'])),
        launch_arguments={
            'namespace': namespace,
            'map': LaunchConfiguration('map'),
            'params': LaunchConfiguration('localization_params'),
            'use_sim_time': use_sim_time,
        }.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_turtlebot4_bringup, 'launch', 'nav2.launch.py'])),
        launch_arguments={
            'namespace': namespace,
            'params_file': LaunchConfiguration('nav2_params'),
            'use_sim_time': use_sim_time,
        }.items())

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rviz)
    ld.add_action(localization)
    ld.add_action(nav2)
    return ld
