#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

from turtlebot4_custom_py.llm_location_mapper import LLMLocationMapper, locations_from_map
from turtlebot4_custom_py.map_locations import load_map_locations


class LLMNavigationNode(Node):
    """
    ROS2 node that listens for natural language navigation commands
    and uses a local GGUF LLM to extract locations and navigate the robot.
    """

    def __init__(self):
        super().__init__('llm_navigation_node')

        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('command_topic', 'navigation_command')
        self.declare_parameter('n_threads', 4)

        # Get parameters
        model_path = self.get_parameter('model_path').value or None
        command_topic = self.get_parameter('command_topic').value
        n_threads = self.get_parameter('n_threads').value

        self.get_logger().info(f'Initializing LLM Navigation Node')
        self.get_logger().info(f'Model path: {model_path or "default"}')

        # Initialize the TurtleBot4 navigator. For a namespaced robot run
        # with --ros-args -r __ns:=/<robot>, which namespaces this node, the
        # navigator, and the command topic together (a robot_namespace
        # parameter would only move the navigator, not this node's topics).
        self.get_logger().info('Initializing TurtleBot4 Navigator...')
        self.navigator = TurtleBot4Navigator()

        # The LLM picks from the active map's surveyed location names, so the
        # localization launch (which runs map_server) must already be up —
        # the same prerequisite navigation itself has.
        locations = load_map_locations(self.navigator)
        self.get_logger().info(f'Using locations from map: {locations.map_yaml}')

        # Initialize the LLM location mapper
        self.get_logger().info('Initializing LLM Location Mapper...')
        self.llm_mapper = LLMLocationMapper(
            model_path=model_path,
            locations=locations_from_map(locations),
            n_threads=n_threads,
        )

        # Listen to string command topic
        self.command_subscription = self.create_subscription(
            String,
            command_topic,
            self.command_callback,
            10
        )

        self.get_logger().info(f'Listening for commands on topic: {command_topic}')
        self.get_logger().info('LLM Navigation Node is ready!')

    def command_callback(self, msg: String):
        """
        Callback for incoming navigation commands.

        Args:
            msg: String message containing natural language command
        """
        command = msg.data
        self.get_logger().info(f'Received command: "{command}"')

        # Extract location using LLM
        result = self.llm_mapper.extract_location(command)

        if result is None:
            self.get_logger().warn(f'Could not extract valid location from command: "{command}"')
            self.get_logger().warn('Available locations:')
            for loc_name in self.llm_mapper.get_all_locations().keys():
                self.get_logger().warn(f'  - {loc_name}')
            return

        x, y, direction = result
        self.get_logger().info(f'Navigating to: x={x}, y={y}, direction={direction}')

        # Create goal pose
        goal_pose = self.navigator.getPoseStamped([x, y], direction)

        # Navigate to the location
        self.get_logger().info('Starting navigation...')
        self.navigator.startToPose(goal_pose)

        self.get_logger().info('Navigation command sent!')


def main(args=None):
    """Main function to run the LLM Navigation Node."""
    rclpy.init(args=args)

    try:
        # Create the node
        llm_nav_node = LLMNavigationNode()

        # Spin the node
        rclpy.spin(llm_nav_node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        # Cleanup
        if 'llm_nav_node' in locals():
            llm_nav_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
