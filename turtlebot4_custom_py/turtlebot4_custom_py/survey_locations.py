#!/usr/bin/env python3
"""Interactive helper for surveying a map's named locations from RViz.

With the localization launch running (so /map_server knows which map is
active), pick the 'Publish Point' tool in RViz: type a name in this terminal,
click that spot on the map, repeat. At the end it prints a `locations:` block
ready to paste into the map's maps/<name>.locations.yaml.

It deliberately never writes the file itself: the dock poses in that file
must be surveyed with the actual robot (see docs/navigate_to_a_goal.md), and
a tool that quietly creates the file with guessed dock poses would hand AMCL
a garbage initial pose later.
"""
import rclpy

from geometry_msgs.msg import PointStamped
from rclpy.node import Node

from turtlebot4_custom_py.map_locations import get_active_map_yaml, sidecar_path


class ClickCollector(Node):
    """Hands back the next RViz 'Publish Point' click."""

    def __init__(self):
        super().__init__('survey_locations')
        self.point = None
        self.create_subscription(
            PointStamped, 'clicked_point', self._on_click, 10)

    def _on_click(self, msg):
        self.point = msg.point

    def next_click(self):
        self.point = None
        while rclpy.ok() and self.point is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.point


def main(args=None):
    rclpy.init(args=args)
    node = ClickCollector()

    map_yaml = get_active_map_yaml(node)
    path = sidecar_path(map_yaml)
    print(f'Surveying locations for the active map: {map_yaml}')
    print(f'They belong in: {path}')
    print("In RViz, select the 'Publish Point' tool (top toolbar).\n")

    entries = []
    while rclpy.ok():
        try:
            name = input('Location name (blank to finish): ').strip()
        except EOFError:
            break
        if not name:
            break
        print(f"  Now click '{name}' on the map in RViz...")
        point = node.next_click()
        if point is None:
            break
        print(f'  {name}: x={point.x:.2f}, y={point.y:.2f}')
        entries.append((name, point))

    if entries:
        print(f'\nPaste into {path} under `locations:` '
              '(clicks carry no heading — fix each direction):')
        for name, p in entries:
            print(f'  {name}: {{x: {p.x:.2f}, y: {p.y:.2f}, direction: north}}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
