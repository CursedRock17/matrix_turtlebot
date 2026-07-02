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

SCAN_TIMEOUT_SEC = 60.0
ROBOT_TIMEOUT_SEC = 60.0
MIN_SCAN_HZ = 4.0  # a healthy RPLIDAR is ~7-10 Hz; below this it's too degraded to navigate on


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


def _scan_rate(navigator, window_sec=3.0):
    """Measured /scan rate (Hz) over a short window."""
    count = [0]
    sub = navigator.create_subscription(
        LaserScan, 'scan', lambda msg: count.__setitem__(0, count[0] + 1),
        qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + window_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(navigator, timeout_sec=0.1)
    finally:
        navigator.destroy_subscription(sub)
    return count[0] / window_sec


def _call_start_motor(navigator):
    """Ask the RPLIDAR to start spinning. Idempotent and safe: /start_motor
    starts the lidar (a no-op if it's already running) and never stops it."""
    client = navigator.create_client(Empty, 'start_motor')
    try:
        if client.wait_for_service(timeout_sec=5.0):
            future = client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(navigator, future, timeout_sec=5.0)
        else:
            navigator.warn('No start_motor service found to start the lidar with')
    finally:
        navigator.destroy_client(client)


def _ensure_lidar_spinning(navigator):
    """Spin the RPLIDAR up PROACTIVELY and confirm scans are flowing.

    The dock powers the lidar down, so instead of relying on an undock to wake
    it, we call /start_motor directly (idempotent — it never stops a running
    lidar). Run up front in undock_and_localize so scans flow before the
    costmap/AMCL need them, and again after every redock. Raises if no scan
    appears even after starting the motor.
    """
    _call_start_motor(navigator)
    if not _scan_arrives(navigator):
        navigator.warn('No laser scans yet — asking the RPLIDAR to start again')
        _call_start_motor(navigator)
        if not _scan_arrives(navigator):
            raise RuntimeError(
                'Still no data on /scan after starting the lidar motor. AMCL '
                'cannot localize without scans, so navigation would hang at '
                '"Waiting for amcl_pose". Check `ros2 topic hz /scan`; if it '
                'stays silent, power-cycle the robot (see docs/troubleshooting.md).')

    # Scan is flowing — confirm it's a usable rate, not a degraded trickle.
    # (This is a startup gate; the patrol loop's runtime scan watchdog guards
    # against the intermittent mid-run stalls.)
    hz = _scan_rate(navigator)
    if hz < MIN_SCAN_HZ:
        navigator.warn(f'/scan is only {hz:.1f} Hz (want >= {MIN_SCAN_HZ:.0f} Hz) — '
                       'lidar looks degraded; navigation will be unreliable')
    else:
        navigator.info(f'/scan healthy at {hz:.1f} Hz')


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
            'powered on and on BaleNet (`ping 192.168.50.224`); '
            'ROS_DISCOVERY_SERVER is set (re-source turtlebot4_bringup/'
            'setup.bash in this terminal); robot topics appear '
            '(`ros2 topic list`). See docs/troubleshooting.md.')


def undock_relocalize(navigator, locations):
    """Undock, confirm the lidar restarted, re-seed AMCL, and wait for Nav2.

    Used at startup AND after every charge cycle: while docked the lidar is off
    and AMCL goes stale, so the robot must re-localize on each undock, not only
    the first one. Blocks until AMCL has a pose and Nav2 is active.
    """
    navigator.info('Undocking to the known off-dock pose')
    navigator.undock()

    # Re-confirm the lidar (idempotent /start_motor) after the undock — needed
    # for the charge-cycle redock, where the dock had powered it down again.
    _ensure_lidar_spinning(navigator)

    # The robot is now stationary just off the dock: tell AMCL where that is.
    initial_pose = navigator.getPoseStamped(*locations.undock_pose)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()


def undock_and_localize(navigator):
    """Spin up the lidar, undock to the known pose, then wait for Nav2.

    Order: confirm the robot is on the wire, start the lidar (so scans flow
    before the costmap/AMCL need them), then dock→undock to the known pose,
    set the initial pose, and block until AMCL has confirmed it against a scan
    and bt_navigator reports active. The robot ends up just off the dock,
    localized and ready to navigate.

    Returns the MapLocations for the active map, so callers can navigate to
    its named locations.
    """
    # Look up the active map's surveyed poses BEFORE moving the robot, so a
    # missing or broken locations file fails here and not mid-motion.
    locations = load_map_locations(navigator)
    navigator.info(f'Loaded location file for map: {locations.map_yaml}')

    # Fail fast if the robot isn't reachable — MUST come before any call that
    # needs robot data (getDockedStatus() blocks forever on dock_status, and
    # nav2 aborts on a missing odom frame, with no hint why — 2026-06-18).
    navigator.info('Waiting for the robot to come on the wire (odom)...')
    _wait_for_robot(navigator)

    # Spin the lidar up FRONT — before the dock/undock dance and before nav2's
    # costmap waits on /scan. /start_motor wakes it even while docked and is
    # idempotent (it never stops a running lidar), so doing this first breaks
    # the costmap <- scan <- lidar startup deadlock (2026-06-24 field finding).
    _ensure_lidar_spinning(navigator)

    # Start from the dock so the robot begins at a known pose.
    if not navigator.getDockedStatus():
        navigator.info('Docking before initialising pose')
        navigator.dock()

    # Undock to the known off-dock pose, then localize and wait for Nav2.
    undock_relocalize(navigator, locations)

    return locations
