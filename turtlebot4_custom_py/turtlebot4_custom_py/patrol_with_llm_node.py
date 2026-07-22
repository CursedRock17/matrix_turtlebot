#!/usr/bin/env python3
"""Patrol loop that accepts natural-language reroutes mid-patrol.

Drives the active map's patrol waypoints like nav_patrol_loop, but between
waypoints it checks /navigation_command (std_msgs/String): a sentence like
"take this to Harold's office" is mapped to a surveyed location by the local
LLM (llm_location_mapper), the robot detours there, then resumes the patrol.

This is the integration rung — prove nav_patrol_loop and llm_navigation
separately before running this (see README.md). Note it still uses the simple
blocking startToPose flow; the patrol hardening (goal timeouts, scan
watchdog, dock retries) lives in nav_patrol_loop and hasn't been folded in
here yet.
"""
import collections
from math import floor
from threading import Lock, Thread
from time import sleep

import rclpy

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

from turtlebot4_custom_py.bump_to_cloud import BumpToCloud
from turtlebot4_custom_py.monitors import BatteryMonitor
from turtlebot4_custom_py.llm_location_mapper import LLMLocationMapper, locations_from_map
from turtlebot4_custom_py.map_locations import load_map_locations
from turtlebot4_custom_py.startup import undock_and_localize

BATTERY_HIGH = 0.95
BATTERY_LOW = 0.30
BATTERY_CRITICAL = 0.1


class PatrolWithLLMNode(Node):
    """
    ROS2 node that patrols waypoints continuously but can be rerouted
    via natural language commands on /navigation_command, then resumes patrol.
    """

    def __init__(self, locations):
        super().__init__('patrol_with_llm_node')

        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('n_threads', 4)

        model_path = self.get_parameter('model_path').value or None
        n_threads = self.get_parameter('n_threads').value

        # Thread-safe command queue
        self.command_queue = collections.deque()

        # Subscribe to navigation commands
        self.command_subscription = self.create_subscription(
            String,
            'navigation_command',
            self.command_callback,
            10
        )

        # Initialize the LLM location mapper on the active map's surveyed
        # names, so reroute commands can only target places this map knows.
        self.get_logger().info('Initializing LLM Location Mapper...')
        self.llm_mapper = LLMLocationMapper(
            model_path=model_path,
            locations=locations_from_map(locations),
            n_threads=n_threads,
        )

        self.get_logger().info('PatrolWithLLMNode ready, listening on /navigation_command')

    def command_callback(self, msg: String):
        command = msg.data
        self.get_logger().info(f'Received reroute command: "{command}"')
        self.command_queue.append(command)


def main(args=None):
    rclpy.init(args=args)

    lock = Lock()
    battery_monitor = BatteryMonitor(lock)

    navigator = TurtleBot4Navigator()
    print("Navigator Made")

    # Load the active map's locations up front (also validates the sidecar
    # file before anything moves) — the LLM needs them at node construction.
    locations = load_map_locations(navigator)

    # Create the patrol+LLM node and spin it in a background thread
    patrol_node = PatrolWithLLMNode(locations)
    patrol_executor = SingleThreadedExecutor()
    patrol_executor.add_node(patrol_node)
    patrol_thread = Thread(target=patrol_executor.spin, daemon=True)
    patrol_thread.start()

    battery_percent = None
    position_index = 0

    # Start battery monitor thread
    thread = Thread(target=battery_monitor.thread_function, daemon=True)
    thread.start()

    # Bumps -> costmap obstacles + proximity beeps (base-stack safety net).
    bump_watch = BumpToCloud()
    Thread(target=bump_watch.thread_function, daemon=True).start()

    # Undock first (the docked robot has no lidar), localize, wait for Nav2
    undock_and_localize(navigator)
    print("Running")
    # Patrol waypoints come from the active map's locations file
    goal_pose = locations.patrol_poses(navigator)
    if not goal_pose:
        navigator.error(f'No patrol list in {locations.map_yaml} locations file')
        return

    while True:
        with lock:
            battery_percent = battery_monitor.battery_percent

        if (battery_percent is not None):
            navigator.info(f'Battery is at {(battery_percent*100):.2f}% charge')

            # Check battery charge level
            if (battery_percent < BATTERY_CRITICAL):
                navigator.error('Battery critically low. Charge or power down')
                break
            elif (battery_percent < BATTERY_LOW):
                # Go near the dock
                navigator.info('Docking for charge')
                navigator.startToPose(
                    navigator.getPoseStamped(*locations.dock_approach_pose))
                navigator.dock()

                if not navigator.getDockedStatus():
                    navigator.error('Robot failed to dock')
                    break

                # Wait until charged
                navigator.info('Charging...')
                battery_percent_prev = 0
                while (battery_percent < BATTERY_HIGH):
                    sleep(15)
                    battery_percent_prev = floor(battery_percent*100)/100
                    with lock:
                        battery_percent = battery_monitor.battery_percent

                    if battery_percent > (battery_percent_prev + 0.01):
                        navigator.info(f'Battery is at {(battery_percent*100):.2f}% charge')

                # Undock
                navigator.undock()
                position_index = 0

            else:
                # Check for reroute command before next patrol waypoint
                if patrol_node.command_queue:
                    command = patrol_node.command_queue.popleft()
                    navigator.info(f'Processing reroute command: "{command}"')

                    result = patrol_node.llm_mapper.extract_location(command)
                    if result is not None:
                        x, y, direction = result
                        navigator.info(f'Rerouting to: x={x}, y={y}')
                        reroute_pose = navigator.getPoseStamped([x, y], direction)
                        navigator.startToPose(reroute_pose)
                        navigator.info('Reroute complete, resuming patrol')
                    else:
                        navigator.warn(f'Could not extract location from: "{command}"')
                else:
                    # Navigate to next patrol waypoint
                    navigator.startToPose(goal_pose[position_index])
                    position_index = (position_index + 1) % len(goal_pose)

    patrol_node.destroy_node()
    battery_monitor.destroy_node()
    bump_watch.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
