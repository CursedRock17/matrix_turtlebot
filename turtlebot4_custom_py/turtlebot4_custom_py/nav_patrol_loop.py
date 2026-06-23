#!/usr/bin/env python3
"""Battery-aware patrol loop, hardened for long unattended runs.

Drives the active map's patrol waypoints in a loop, returning to the dock to
charge when low. Built to survive a multi-hour, multi-charge run:
  - every goal has a timeout and a pass/fail check, so a blocked or unreachable
    waypoint is skipped, never wedges the loop;
  - the robot re-localizes after every charge (the lidar is off while docked,
    so AMCL goes stale);
  - charging has a timeout, and a missing battery reading never crashes the loop.
"""
import time
from threading import Lock, Thread

import rclpy

from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import BatteryState
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

from turtlebot4_custom_py.startup import undock_and_localize, undock_relocalize

BATTERY_HIGH = 0.95
BATTERY_LOW = 0.30        # return to the dock to charge below this
BATTERY_CRITICAL = 0.12   # stop the patrol below this

GOAL_TIMEOUT_SEC = 180.0      # cancel a single goal that overruns this
MAX_CHARGE_SEC = 7200.0       # stop waiting for a full charge after this (2 h)
FAILED_GOAL_BACKOFF_SEC = 5.0


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


def main(args=None):
    rclpy.init(args=args)

    lock = Lock()
    battery_monitor = BatteryMonitor(lock)
    Thread(target=battery_monitor.thread_function, daemon=True).start()

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
            navigate_to(navigator,
                        navigator.getPoseStamped(*locations.dock_approach_pose),
                        'dock approach')
            navigator.dock()
            if not navigator.getDockedStatus():
                navigator.error('Failed to dock for charging. Stopping patrol.')
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

        # Normal patrol step.
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
    rclpy.shutdown()


if __name__ == '__main__':
    main()
