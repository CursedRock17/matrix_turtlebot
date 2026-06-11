#!/usr/bin/env python3
"""Shared robot startup sequence: undock first, then localize.

The Turtlebot4 stops the RPLIDAR while docked, and without laser scans AMCL
never publishes amcl_pose, which waitUntilNav2Active() blocks on. So every
node must undock BEFORE localizing — see docs/navigate_to_a_goal.md.
"""
from turtlebot4_custom_py.map_locations import load_map_locations


def undock_and_localize(navigator):
    """Start from the dock, undock to spin up the lidar, then wait for Nav2.

    Blocks until AMCL has confirmed the pose against a laser scan and
    bt_navigator reports active. The robot ends up just off the dock,
    localized and ready to navigate.

    Returns the MapLocations for the active map, so callers can navigate to
    its named locations.
    """
    # Look up the active map's surveyed poses BEFORE moving the robot, so a
    # missing or broken locations file fails here and not mid-motion.
    locations = load_map_locations(navigator)
    navigator.info(f'Loaded location file for map: {locations.map_yaml}')

    # Start from the dock so the robot begins at a known pose.
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    navigator.info('Undocking so the lidar is running before localizing')
    navigator.undock()

    # The robot is now stationary just off the dock: tell AMCL where that is.
    initial_pose = navigator.getPoseStamped(*locations.undock_pose)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()
    return locations
