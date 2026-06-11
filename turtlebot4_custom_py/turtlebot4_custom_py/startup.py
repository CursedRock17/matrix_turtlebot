#!/usr/bin/env python3
"""Shared robot startup sequence: undock first, then localize.

The Turtlebot4 stops the RPLIDAR while docked, and without laser scans AMCL
never publishes amcl_pose, which waitUntilNav2Active() blocks on. So every
node must undock BEFORE localizing — see docs/navigate_to_a_goal.md.
"""
import time

import rclpy

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

from turtlebot4_custom_py.map_locations import load_map_locations

SCAN_TIMEOUT_SEC = 10.0


def _scan_arrives(navigator, timeout_sec=SCAN_TIMEOUT_SEC):
    """True once a message shows up on scan within the timeout."""
    got = []
    sub = navigator.create_subscription(
        LaserScan, 'scan', lambda msg: got.append(True),
        qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + timeout_sec
        while not got and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(navigator, timeout_sec=0.25)
        return bool(got)
    finally:
        navigator.destroy_subscription(sub)


def _ensure_lidar_spinning(navigator):
    """Make sure the RPLIDAR actually restarted after the undock.

    The lidar is stopped while docked by design, but after a redock it has
    been seen to stay stopped on undock (bags/fifth_bag; turtlebot4 issues
    #392/#658). Without scans AMCL waits forever and the startup just hangs
    silently, so check for scan data and try a motor restart before giving
    an actionable error.
    """
    if _scan_arrives(navigator):
        return

    navigator.warn('No laser scans after undocking — asking the RPLIDAR to restart')
    client = navigator.create_client(Empty, 'start_motor')
    try:
        if client.wait_for_service(timeout_sec=5.0):
            future = client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(navigator, future, timeout_sec=5.0)
        else:
            navigator.warn('No start_motor service found to restart the lidar with')
    finally:
        navigator.destroy_client(client)

    if not _scan_arrives(navigator):
        raise RuntimeError(
            'Still no data on scan after undocking and restarting the lidar '
            'motor. AMCL cannot localize without scans, so navigation would '
            'hang at "Waiting for amcl_pose". Check `ros2 topic hz /scan`; '
            'if it stays silent, power-cycle the robot (known redock issue, '
            'see docs/troubleshooting.md).')


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

    # Don't localize until the lidar is confirmed back up — after a redock
    # it sometimes is not, and AMCL would silently wait forever.
    _ensure_lidar_spinning(navigator)

    # The robot is now stationary just off the dock: tell AMCL where that is.
    initial_pose = navigator.getPoseStamped(*locations.undock_pose)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()
    return locations
