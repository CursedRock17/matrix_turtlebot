#!/usr/bin/env python3
"""Shared robot startup sequence: undock first, then localize.

The Turtlebot4 stops the RPLIDAR while docked, and without laser scans AMCL
never publishes amcl_pose, which waitUntilNav2Active() blocks on. So every
node must undock BEFORE localizing — see docs/navigate_to_a_goal.md.
"""
import time

import rclpy

from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

from turtlebot4_custom_py.map_locations import load_map_locations

SCAN_TIMEOUT_SEC = 10.0
ROBOT_TIMEOUT_SEC = 10.0


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


def _wait_for_robot(navigator, timeout_sec=ROBOT_TIMEOUT_SEC):
    """Fail fast if the robot isn't on the wire.

    Every robot topic (dock_status, odom, scan, ...) is published by the
    Create 3 / Pi, so if none arrive the laptop simply isn't talking to the
    robot — unsourced/wrong ROS_DISCOVERY_SERVER, robot powered off, or off
    BaleNet. Without this check the node blocks silently for minutes inside
    getDockedStatus() (waiting on dock_status) while nav2 separately aborts on
    a missing odom frame, with no hint why (2026-06-18 field session).

    We wait on odom because the Create 3 publishes it continuously whether
    docked or not (unlike scan, which is off while docked).
    """
    got = []
    sub = navigator.create_subscription(
        Odometry, 'odom', lambda msg: got.append(True),
        qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + timeout_sec
        while not got and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(navigator, timeout_sec=0.25)
    finally:
        navigator.destroy_subscription(sub)
    if not got:
        raise RuntimeError(
            f'No data from the robot after {timeout_sec:.0f}s (nothing on '
            '/odom). The laptop is not talking to the TurtleBot4. Check: robot '
            'powered on and on BaleNet (`ping 192.168.50.223`); '
            'ROS_DISCOVERY_SERVER is set (re-source turtlebot4_bringup/'
            'setup.bash in this terminal); robot topics appear '
            '(`ros2 topic list`). See docs/troubleshooting.md.')


def undock_relocalize(navigator, locations):
    """Undock, confirm the lidar restarted, re-seed AMCL, and wait for Nav2.

    Used at startup AND after every charge cycle: while docked the lidar is off
    and AMCL goes stale, so the robot must re-localize on each undock, not only
    the first one. Blocks until AMCL has a pose and Nav2 is active.
    """
    navigator.info('Undocking so the lidar is running before localizing')
    navigator.undock()

    # Don't localize until the lidar is confirmed back up — after a redock it
    # sometimes is not, and AMCL would silently wait forever.
    _ensure_lidar_spinning(navigator)

    # The robot is now stationary just off the dock: tell AMCL where that is.
    initial_pose = navigator.getPoseStamped(*locations.undock_pose)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()


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

    # Fail fast if the robot isn't reachable — this MUST come before any call
    # that needs robot data. getDockedStatus() below blocks forever waiting on
    # dock_status, and nav2 separately aborts on a missing odom frame, with no
    # hint why (2026-06-18 field session: robot went silent after a power-cycle
    # and the node dead-ended at "Loaded location file").
    navigator.info('Waiting for the robot to come on the wire (odom)...')
    _wait_for_robot(navigator)

    # Start from the dock so the robot begins at a known pose.
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    undock_relocalize(navigator, locations)

    return locations
