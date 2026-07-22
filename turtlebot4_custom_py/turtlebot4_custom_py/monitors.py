#!/usr/bin/env python3
"""Background monitor nodes shared by the navigation examples.

Each monitor is a tiny Node designed to spin in its own daemon thread
(via thread_function) so the main script keeps its blocking navigator
flow while the monitors track robot state in the background:

    monitor = BatteryMonitor(lock)
    Thread(target=monitor.thread_function, daemon=True).start()
"""
import time
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, LaserScan


class BatteryMonitor(Node):
    """Tracks the latest /battery_state percentage.

    battery_percent is None until the first message arrives — callers must
    handle that (wait, don't crash). Read/write it under the lock passed in,
    which the caller shares with its own loop.
    """

    def __init__(self, lock):
        super().__init__('battery_monitor')
        self.lock = lock
        self.battery_percent = None
        self.battery_state_subscriber = self.create_subscription(
            BatteryState, 'battery_state', self.battery_state_callback,
            qos_profile_sensor_data)

    def battery_state_callback(self, batt_msg: BatteryState):
        with self.lock:
            self.battery_percent = batt_msg.percentage

    def thread_function(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()


class ScanWatchdog(Node):
    """Tracks /scan arrival AND timestamp skew so a patrol won't drive blind.

    Two distinct failure modes, both of which leave the robot without lidar
    protection while looking superficially alive:
      - arrival stalls: the lidar spins but delivery stalls 1-2 s
        (Wi-Fi / laptop CPU; 2026-06-23 bag) -> scan_age() grows;
      - clock skew: scans FLOW at a healthy rate but their stamps are seconds
        off the laptop clock, so collision_monitor and the costmap silently
        drop every one of them ("timestamps differ ... Ignoring the source",
        2026-07-01 run) -> scan_skew() grows while scan_age() stays tiny.
    last_scan/last_skew are bare float assignments, safe to read without a
    lock under the GIL.
    """

    def __init__(self):
        super().__init__('scan_watchdog')
        self.last_scan = None
        self.last_skew = None
        self.create_subscription(
            LaserScan, 'scan', self._scan_callback, qos_profile_sensor_data)

    def _scan_callback(self, msg):
        self.last_scan = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0.0:
            now = self.get_clock().now().nanoseconds * 1e-9
            self.last_skew = now - stamp

    def scan_age(self):
        """Seconds since the last scan arrived (None until one does)."""
        return None if self.last_scan is None else time.monotonic() - self.last_scan

    def scan_skew(self):
        """Laptop clock minus latest scan stamp, in seconds (None until a scan
        arrives). Healthy is ~0.01-0.05 s of transport delay; seconds means a
        robot clock has diverged and nav2 is ignoring the lidar."""
        return self.last_skew

    def thread_function(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()
