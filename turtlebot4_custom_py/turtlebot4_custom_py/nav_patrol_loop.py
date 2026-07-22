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
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

from turtlebot4_custom_py.bump_to_cloud import BumpToCloud
from turtlebot4_custom_py.monitors import BatteryMonitor, ScanWatchdog
from turtlebot4_custom_py.startup import undock_and_localize, undock_relocalize

BATTERY_HIGH = 0.95
BATTERY_LOW = 0.30        # return to the dock to charge below this
BATTERY_CRITICAL = 0.12   # stop the patrol below this

GOAL_TIMEOUT_SEC = 300.0      # cancel a goal that overruns this (building legs are slow)
MAX_CHARGE_SEC = 7200.0       # stop waiting for a full charge after this (2 h)
FAILED_GOAL_BACKOFF_SEC = 5.0
DOCK_ATTEMPTS = 3             # re-approach and retry a missed dock this many times

SCAN_STALE_SEC = 2.0           # /scan older than this means we'd be driving blind
SCAN_SKEW_SEC = 2.0            # scan stamps this far off the laptop clock get silently
                               # DROPPED by collision_monitor/costmap (2026-07-01 run)
SCAN_HOLD_TIMEOUT_SEC = 120.0  # hold this long for /scan to recover before skipping the goal


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


def dock_for_charging(navigator, locations, watchdog):
    """Dock with retries. Returns True once actually docked.

    The Create 3's IR docking behavior only finds the dock when it starts
    close enough (~1 m) with the dock in view, and nav2 can deliver us to the
    approach pose with enough error to miss that window (2026-07-01 run ended
    with 'Failed to dock for charging'). We can't make the firmware search
    longer, but re-navigating to the approach pose re-rolls the localization/
    controller error, so a fresh approach usually lands inside the window.

    2026-07-02 run: all three approach navigations FAILED (a scan stall starved
    AMCL right at the dock) and the old code called dock() anyway from wherever
    the abort left the robot, burning every attempt outside the IR window. So:
    hold for a usable scan before each approach, and only spend a dock() on an
    approach that actually succeeded — except the last attempt, where a
    ~25 s hail-mary dock() beats ending the patrol.
    """
    for attempt in range(1, DOCK_ATTEMPTS + 1):
        # A scan stall poisons AMCL/nav2 exactly when the approach needs them;
        # holding here is what lets a retry succeed after a failed approach.
        wait_for_fresh_scan(navigator, watchdog)
        arrived = navigate_to(navigator,
                              navigator.getPoseStamped(*locations.dock_approach_pose),
                              f'dock approach ({attempt}/{DOCK_ATTEMPTS})')
        if not arrived and attempt < DOCK_ATTEMPTS:
            navigator.warn(f'Dock approach {attempt}/{DOCK_ATTEMPTS} failed — '
                           'not docking blind; holding for scan, then retrying')
            time.sleep(FAILED_GOAL_BACKOFF_SEC)
            continue
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

    # Bumps -> costmap obstacles + proximity beeps, part of the base stack
    # (feet are below the lidar plane; the bumper is the only sensor for them).
    bump_watch = BumpToCloud()
    Thread(target=bump_watch.thread_function, daemon=True).start()

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
            if not dock_for_charging(navigator, locations, scan_watchdog):
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
    bump_watch.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
