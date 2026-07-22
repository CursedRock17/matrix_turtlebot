#!/usr/bin/env python3
"""Undock, localize from the known dock pose, and navigate to a goal.

The smallest end-to-end navigation example (see README.md): if this works,
bringup, time sync, discovery, and the map's locations file are all good.
The goal below is map-frame coordinates on the ACTIVE map — pick a free spot
with RViz 'Publish Point' and edit the constants.
"""
from threading import Thread

import rclpy

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

from turtlebot4_custom_py.bump_to_cloud import BumpToCloud
from turtlebot4_custom_py.startup import undock_and_localize

GOAL_POSITION = [2.65, -1.625]
GOAL_DIRECTION = TurtleBot4Directions.EAST


def main():
    """Run the dock -> undock -> localize -> navigate sequence."""
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Bumps -> costmap obstacles + proximity beeps (base-stack safety net).
    bump_watch = BumpToCloud()
    Thread(target=bump_watch.thread_function, daemon=True).start()

    undock_and_localize(navigator)

    goal_pose = navigator.getPoseStamped(GOAL_POSITION, GOAL_DIRECTION)
    print('Undocked and localized — driving to', GOAL_POSITION)
    navigator.startToPose(goal_pose)  # blocks until the goal resolves

    rclpy.shutdown()


if __name__ == '__main__':
    main()
