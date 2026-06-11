#!/usr/bin/env python3
"""Undock, localize from the known dock pose, and navigate to a goal."""
import rclpy

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

from turtlebot4_custom_py.startup import undock_and_localize

GOAL_POSITION = [2.65, -1.625]
GOAL_DIRECTION = TurtleBot4Directions.EAST


def main():
    """Run the dock -> undock -> localize -> navigate sequence."""
    rclpy.init()

    navigator = TurtleBot4Navigator()

    undock_and_localize(navigator)

    print("We've Undocked")
    goal_pose = navigator.getPoseStamped(GOAL_POSITION, GOAL_DIRECTION)
    
    print("Goal Pose: ", goal_pose)
    navigator.startToPose(goal_pose)

    print("Going to Goal")
    rclpy.shutdown()
