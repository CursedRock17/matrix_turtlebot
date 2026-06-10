#!/usr/bin/env python3

import collections
from math import floor
from threading import Lock, Thread
from time import sleep

import rclpy

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

from turtlebot4_custom_py.nav_patrol_loop import BatteryMonitor
from turtlebot4_custom_py.llm_location_mapper import LLMLocationMapper
from turtlebot4_custom_py.startup import undock_and_localize

BATTERY_HIGH = 0.95
BATTERY_LOW = 0.30
BATTERY_CRITICAL = 0.1


class PatrolWithLLMNode(Node):
    """
    ROS2 node that patrols waypoints continuously but can be rerouted
    via natural language commands on /navigation_command, then resumes patrol.
    """

    def __init__(self):
        super().__init__('patrol_with_llm_node')

        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('n_threads', 4)
        self.declare_parameter('robot_namespace', '')

        model_path = self.get_parameter('model_path').value or None
        n_threads = self.get_parameter('n_threads').value
        robot_namespace = self.get_parameter('robot_namespace').value

        # Thread-safe command queue
        self.command_queue = collections.deque()

        # Subscribe to navigation commands
        self.command_subscription = self.create_subscription(
            String,
            'navigation_command',
            self.command_callback,
            10
        )

        # Initialize the LLM location mapper
        self.get_logger().info('Initializing LLM Location Mapper...')
        self.llm_mapper = LLMLocationMapper(
            model_path=model_path,
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

    # Create the patrol+LLM node and spin it in a background thread
    patrol_node = PatrolWithLLMNode()
    patrol_executor = SingleThreadedExecutor()
    patrol_executor.add_node(patrol_node)
    patrol_thread = Thread(target=patrol_executor.spin, daemon=True)
    patrol_thread.start()

    battery_percent = None
    position_index = 0

    # Start battery monitor thread
    thread = Thread(target=battery_monitor.thread_function, daemon=True)
    thread.start()

    # Undock first (the docked robot has no lidar), localize, wait for Nav2
    undock_and_localize(navigator)
    print("Running")
    # Prepare patrol waypoints
    goal_pose = []
    goal_pose.append(navigator.getPoseStamped([-0.30, 2.9], TurtleBot4Directions.NORTH_WEST))
    goal_pose.append(navigator.getPoseStamped([0.57, 0.25], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped([0.89, 0.97], TurtleBot4Directions.NORTH))

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
                navigator.startToPose(navigator.getPoseStamped([-1.0, 1.0],
                                      TurtleBot4Directions.EAST))
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
    rclpy.shutdown()


if __name__ == '__main__':
    main()
