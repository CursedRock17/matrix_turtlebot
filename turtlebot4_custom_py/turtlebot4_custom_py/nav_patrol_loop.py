#!/usr/bin/env python3
"""Battery-aware patrol loop, hardened for long unattended runs.

Drives the active map's patrol waypoints in a loop, returning to the dock to
charge when low. Built to survive a multi-hour, multi-charge run:
  - every goal has a timeout and a pass/fail check, so a blocked or unreachable
    waypoint is skipped, never wedges the loop;
  - the robot re-localizes after every charge (the lidar is off while docked,
    so AMCL goes stale);
  - it won't issue a goal while /scan is stale OR its stamps are seconds off
    the laptop clock (nav2 silently ignores skewed scans — same blindness),
    holding until the scan recovers;
  - a missed dock is retried from a fresh approach instead of ending the run;
  - charging has a timeout, and a missing battery reading never crashes the loop.
"""
import time
from threading import Lock, Thread

import rclpy

from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import BatteryState, LaserScan
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

from turtlebot4_custom_py.startup import undock_and_localize, undock_relocalize

BATTERY_HIGH = 0.95
BATTERY_LOW = 0.30        # return to the dock to charge below this
BATTERY_CRITICAL = 0.12   # stop the patrol below this

GOAL_TIMEOUT_SEC = 300.0      # cancel a single goal that overruns this (building-scale legs are slow)
MAX_CHARGE_SEC = 7200.0       # stop waiting for a full charge after this (2 h)
FAILED_GOAL_BACKOFF_SEC = 5.0
DOCK_ATTEMPTS = 3             # re-approach and retry a missed dock this many times

SCAN_STALE_SEC = 2.0           # /scan older than this means we'd be driving blind
SCAN_SKEW_SEC = 2.0            # scan stamps this far off the laptop clock get silently
                               # DROPPED by collision_monitor/costmap (2026-07-01 run)
SCAN_HOLD_TIMEOUT_SEC = 120.0  # hold this long for /scan to recover before skipping the goal


class BatteryMonitor(Node):

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
    """Tracks /scan arrival AND timestamp skew so the patrol won't drive blind.

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


def _scan_usable(watchdog):
    """True when /scan is both arriving AND stamped close to the laptop clock.
    Either failure means nav2 has no usable lidar, even if the topic 'works'."""
    age = watchdog.scan_age()
    skew = watchdog.scan_skew()
    fresh = age is not None and age <= SCAN_STALE_SEC
    synced = skew is None or abs(skew) <= SCAN_SKEW_SEC
    return fresh and synced


def wait_for_fresh_scan(navigator, watchdog):
    """Block until /scan is fresh and time-synced, so we never drive blind.

    Returns True once scan is usable; False if it stays bad past
    SCAN_HOLD_TIMEOUT_SEC (caller skips the goal). A stall that begins mid-goal
    is still handled by nav2 (it aborts on the stale transform); this guards the
    moment we issue a new goal.
    """
    if _scan_usable(watchdog):
        return True
    age, skew = watchdog.scan_age(), watchdog.scan_skew()
    if skew is not None and abs(skew) > SCAN_SKEW_SEC:
        navigator.warn(f'/scan stamps are {skew:+.1f}s off the laptop clock — '
                       'collision_monitor/costmap are IGNORING the lidar (clock '
                       'skew, see docs/time_sync.md). Holding until it converges.')
    else:
        navigator.warn(f'/scan stale (age={age}) — holding, will not drive blind')
    deadline = time.monotonic() + SCAN_HOLD_TIMEOUT_SEC
    while time.monotonic() < deadline and rclpy.ok():
        time.sleep(0.5)
        if _scan_usable(watchdog):
            navigator.info('/scan recovered — resuming patrol')
            return True
    navigator.error(f'/scan unusable for {SCAN_HOLD_TIMEOUT_SEC:.0f}s '
                    f'(age={watchdog.scan_age()}, skew={watchdog.scan_skew()}) — '
                    'check the lidar, Wi-Fi/CPU load, and chrony on the Pi')
    return False


def navigate_to(navigator, pose, label, timeout_sec=GOAL_TIMEOUT_SEC):
    """Drive to one goal with a timeout. Returns True on success, never raises.

    A rejected, failed, or timed-out goal is cancelled and reported so the
    patrol can move on instead of hanging on a blocked or unreachable waypoint.
    """
    if not navigator.goToPose(pose):
        navigator.warn(f'Goal "{label}" was rejected by nav2')
        return False
    start = time.monotonic()
    while not navigator.isTaskComplete():
        if time.monotonic() - start > timeout_sec:
            navigator.warn(f'Goal "{label}" exceeded {timeout_sec:.0f}s — cancelling')
            navigator.cancelTask()
            return False
        time.sleep(0.5)
    result = navigator.getResult()
    if result != TaskResult.SUCCEEDED:
        navigator.warn(f'Goal "{label}" ended with {result}')
        return False
    return True


def dock_for_charging(navigator, locations):
    """Dock with retries. Returns True once actually docked.

    The Create 3's IR docking behavior only finds the dock when it starts
    close enough (~1 m) with the dock in view, and nav2 can deliver us to the
    approach pose with enough error to miss that window (2026-07-01 run ended
    with 'Failed to dock for charging'). We can't make the firmware search
    longer, but re-navigating to the approach pose re-rolls the localization/
    controller error, so a fresh approach usually lands inside the window.
    """
    for attempt in range(1, DOCK_ATTEMPTS + 1):
        navigate_to(navigator,
                    navigator.getPoseStamped(*locations.dock_approach_pose),
                    f'dock approach ({attempt}/{DOCK_ATTEMPTS})')
        navigator.dock()  # blocks until the dock action finishes
        if navigator.getDockedStatus():
            return True
        navigator.warn(f'Dock attempt {attempt}/{DOCK_ATTEMPTS} missed — '
                       'backing off to the approach pose to retry')
    return False


def main(args=None):
    rclpy.init(args=args)

    lock = Lock()
    battery_monitor = BatteryMonitor(lock)
    Thread(target=battery_monitor.thread_function, daemon=True).start()

    scan_watchdog = ScanWatchdog()
    Thread(target=scan_watchdog.thread_function, daemon=True).start()

    navigator = TurtleBot4Navigator()
    print("Navigator Made")

    # Undock, localize, wait for Nav2 (raises if the robot isn't on the wire).
    locations = undock_and_localize(navigator)
    goal_pose = locations.patrol_poses(navigator)
    names = locations.patrol
    if not goal_pose:
        navigator.error(f'No patrol list in {locations.map_yaml} locations file')
        return
    navigator.info(f'Patrolling {len(goal_pose)} waypoints: {", ".join(names)}')

    position_index = 0
    loop_count = 0
    consecutive_failures = 0

    while rclpy.ok():
        with lock:
            battery_percent = battery_monitor.battery_percent

        # No battery reading yet — wait rather than crash on None.
        if battery_percent is None:
            time.sleep(1.0)
            continue

        if battery_percent < BATTERY_CRITICAL:
            navigator.error(f'Battery critically low ({battery_percent*100:.0f}%). '
                            'Stopping patrol.')
            break

        if battery_percent < BATTERY_LOW:
            navigator.info(f'Battery {battery_percent*100:.0f}% — returning to dock')
            if not dock_for_charging(navigator, locations):
                navigator.error(f'Failed to dock after {DOCK_ATTEMPTS} attempts. '
                                'Stopping patrol.')
                break

            navigator.info('Charging...')
            charge_deadline = time.monotonic() + MAX_CHARGE_SEC
            while battery_percent < BATTERY_HIGH and time.monotonic() < charge_deadline:
                time.sleep(15)
                with lock:
                    battery_percent = battery_monitor.battery_percent
                if battery_percent is None:
                    battery_percent = 0.0
            navigator.info(f'Resuming patrol at {battery_percent*100:.0f}% charge')

            # Re-localize: the lidar was off while docked, so AMCL is stale.
            undock_relocalize(navigator, locations)
            position_index = 0
            consecutive_failures = 0
            continue

        # Normal patrol step — don't issue a goal while /scan is stale (blind).
        if not wait_for_fresh_scan(navigator, scan_watchdog):
            continue
        label = names[position_index]
        navigator.info(f'[loop {loop_count}] -> "{label}" '
                       f'(battery {battery_percent*100:.0f}%)')
        if navigate_to(navigator, goal_pose[position_index], label):
            consecutive_failures = 0
        else:
            consecutive_failures += 1
            if consecutive_failures >= len(goal_pose):
                navigator.warn(f'{consecutive_failures} goals failed in a row — '
                               'check for a blockage or lost localization')
            time.sleep(FAILED_GOAL_BACKOFF_SEC)

        position_index += 1
        if position_index >= len(goal_pose):
            position_index = 0
            loop_count += 1

    battery_monitor.destroy_node()
    scan_watchdog.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
