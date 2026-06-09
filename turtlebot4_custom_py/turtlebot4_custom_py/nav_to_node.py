#!/usr/bin/env python3
import rclpy

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

def main():
    rclpy.init()

    robo_namespace = "/matrix_turtlebot1"
    navigator = TurtleBot4Navigator()

    print("Navigator Made")
    # Start on dock
    if not navigator.getDockedStatus():
        print("Docking?")
        navigator.info('Docking before intialising pose')
        navigator.dock()

    print("We've Docked")
    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    print("Set Initial")
    # Wait for Nav2
    navigator.waitUntilNav2Active()
    #navigator=(robo_namespace + '/' + 'bt_navigator'),
    #localizer=(robo_namespace + '/' + 'amcl'),

    print("Nav Ready")
    # Set goal poses
    goal_pose = navigator.getPoseStamped([1.50, -0.75], TurtleBot4Directions.EAST)

    print("Running")
    # Undock
    navigator.undock()

    print("Undock")
    # Go to each goal pose
    navigator.startToPose(goal_pose)

    print("Going")
    rclpy.shutdown()
