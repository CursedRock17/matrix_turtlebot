#!/usr/bin/env python3
"""Shared robot startup sequence: undock first, then localize.

The Turtlebot4 stops the RPLIDAR while docked, and without laser scans AMCL
never publishes amcl_pose, which waitUntilNav2Active() blocks on. So every
node must undock BEFORE localizing — see docs/navigate_to_a_goal.md.
"""
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions

# Our maps are built by SLAM starting on the dock, so the dock is the map
# origin, facing NORTH (+x). Undocking backs the robot ~0.5 m off the dock
# and rotates it 180 degrees, which puts it here:
UNDOCKED_POSITION = [-0.5, 0.0]
UNDOCKED_DIRECTION = TurtleBot4Directions.SOUTH


def undock_and_localize(navigator):
    """Start from the dock, undock to spin up the lidar, then wait for Nav2.

    Blocks until AMCL has confirmed the pose against a laser scan and
    bt_navigator reports active. The robot ends up just off the dock,
    localized and ready to navigate.
    """
    # Start from the dock so the robot begins at a known pose.
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    navigator.info('Undocking so the lidar is running before localizing')
    navigator.undock()

    # The robot is now stationary just off the dock: tell AMCL where that is.
    initial_pose = navigator.getPoseStamped(UNDOCKED_POSITION, UNDOCKED_DIRECTION)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()
