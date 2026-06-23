#!/usr/bin/env python3
"""Preflight 'doctor' for the TurtleBot4: is the robot actually on the wire?

Run this BEFORE launching the nav stack. It checks the laptop->robot link and
records a short diagnostic bag, so a failed startup always leaves the evidence
that tells "robot silent" apart from a real nav bug. The recurring
robot-silent-after-power-cycle case (2026-06-18) showed every robot topic at
0 messages; this catches it in seconds instead of a 60 s nav2 abort.

    ros2 run turtlebot4_custom_py preflight
    ros2 run turtlebot4_custom_py preflight --ros-args -p window_sec:=8.0

Exits 0 if the robot is publishing, 1 if not (usable as a launch gate).
"""
import os
import subprocess
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, LaserScan

ROBOT_IP = '192.168.50.223'
# Recorded to the diagnostic bag generically, so the ones we don't subscribe
# to live (tf, dock_status, ...) need no message imports here.
BAG_TOPICS = ['/odom', '/tf', '/tf_static', '/scan', '/dock_status',
              '/battery_state', '/amcl_pose', '/initialpose']

OK, FAIL, WARN = '[ OK ]', '[FAIL]', '[WARN]'


class Preflight(Node):

    def __init__(self):
        super().__init__('preflight_doctor')
        self.declare_parameter('robot_ip', ROBOT_IP)
        self.declare_parameter('window_sec', 6.0)
        self.declare_parameter('bag_dir', 'claude_logs')

        self.counts = {'/odom': 0, '/battery_state': 0, '/scan': 0}
        self.odom_delay_sum = 0.0
        self.create_subscription(
            Odometry, 'odom', self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(
            BatteryState, 'battery_state',
            lambda m: self._bump('/battery_state'), qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, 'scan',
            lambda m: self._bump('/scan'), qos_profile_sensor_data)

    def _bump(self, topic):
        self.counts[topic] += 1

    def _odom_cb(self, msg):
        self.counts['/odom'] += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0.0:
            now = self.get_clock().now().nanoseconds * 1e-9
            self.odom_delay_sum += abs(now - stamp)


def _run(cmd, timeout):
    """Run a shell command, returning the CompletedProcess or None on failure."""
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None


def main(args=None):
    rclpy.init(args=args)
    node = Preflight()
    ip = node.get_parameter('robot_ip').value
    window = float(node.get_parameter('window_sec').value)
    bag_dir = node.get_parameter('bag_dir').value

    checks = []  # (status, text)

    ds = os.environ.get('ROS_DISCOVERY_SERVER', '')
    checks.append((OK, f'ROS_DISCOVERY_SERVER = {ds}') if ds else
                  (WARN, 'ROS_DISCOVERY_SERVER is empty — source '
                         'turtlebot4_bringup/setup.bash in this terminal'))

    ss = _run(['ss', '-lun'], 3)
    if ss is not None:
        up = ':11888' in ss.stdout
        checks.append((OK, 'local discovery server :11888 listening') if up else
                      (WARN, 'local discovery server :11888 NOT listening — '
                             're-source setup.bash'))

    ping = _run(['ping', '-c', '2', '-W', '2', ip], 6)
    reachable = ping is not None and ping.returncode == 0
    checks.append((OK, f'robot {ip} reachable') if reachable else
                  (FAIL, f'robot {ip} NOT reachable — powered on and on BaleNet?'))

    # Record a diagnostic bag while we listen, so a failure leaves evidence.
    os.makedirs(bag_dir, exist_ok=True)
    bag_path = os.path.join(bag_dir, 'preflight_' + datetime.now().strftime('%H%M%S'))
    bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', bag_path, *BAG_TOPICS],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    node.get_logger().info(f'Listening {window:.0f}s for robot topics...')
    deadline = time.monotonic() + window
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    bag.send_signal(2)  # SIGINT -> clean bag close
    try:
        bag.wait(timeout=5)
    except subprocess.TimeoutExpired:
        bag.kill()

    c = node.counts
    odom_ok = c['/odom'] > 0
    checks.append((OK, f"/odom: {c['/odom']} msgs ({c['/odom']/window:.0f} Hz)") if odom_ok
                  else (FAIL, '/odom: 0 msgs — ROBOT SILENT; nav2 will abort on a '
                              'missing odom frame and getDockedStatus() will hang'))
    checks.append((OK, f"/battery_state: {c['/battery_state']} msgs") if c['/battery_state']
                  else (WARN, '/battery_state: 0 msgs'))
    # scan is off while docked, so its absence alone is not a failure.
    checks.append((OK, f"/scan: {c['/scan']} msgs") if c['/scan'] else
                  (WARN, '/scan: 0 msgs (normal while docked; lidar starts on undock)'))
    if odom_ok:
        avg = node.odom_delay_sum / c['/odom']
        checks.append((OK, f'odom timestamp delay ~{avg*1000:.0f} ms') if avg < 0.05 else
                      (WARN, f'odom timestamp delay ~{avg*1000:.0f} ms — check time sync '
                             '(docs/time_sync.md)'))

    healthy = reachable and odom_ok
    print('\n===== TurtleBot4 preflight =====')
    for status, text in checks:
        print(f'  {status} {text}')
    print('  -------------------------------')
    print('  VERDICT: ' + ('robot is on the wire — clear to launch' if healthy else
                           'ROBOT NOT READY — fix the [FAIL] lines above'))
    print(f'  diagnostic bag: {bag_path}')
    print('================================\n')

    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if healthy else 1


if __name__ == '__main__':
    raise SystemExit(main())
