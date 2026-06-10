#!/usr/bin/env python3

"""Frontier exploration node for TurtleBot4.

Subscribes to the occupancy grid map, detects frontiers (boundaries between
known free space and unknown space), clusters them, picks the best one to
explore, and navigates there using Nav2. Repeats until the map is fully
explored or battery is low.
"""

import math
from collections import deque
from threading import Lock, Thread
from time import sleep

import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from rclpy.qos import qos_profile_sensor_data

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

from turtlebot4_custom_py.startup import undock_and_localize

# Map cell values
FREE = 0        # Known free space (0 in OccupancyGrid)
UNKNOWN = -1    # Unknown space
OCCUPIED = 100  # Obstacle

# Exploration tuning
MIN_FRONTIER_SIZE = 5            # Minimum cells in a frontier cluster to consider
GOAL_REACHED_TOLERANCE = 0.5     # Meters — close enough to a frontier goal
EXPLORATION_TIMEOUT = 120.0      # Seconds to wait for navigation to a frontier
MIN_GOAL_DISTANCE = 0.3          # Don't pick frontiers closer than this (meters)

# Battery thresholds
BATTERY_HIGH = 0.95
BATTERY_LOW = 0.30
BATTERY_CRITICAL = 0.10


class MapSubscriber(Node):
    """Subscribes to /map and stores the latest occupancy grid."""

    def __init__(self, lock):
        super().__init__('frontier_map_subscriber')
        self.lock = lock
        self.map_data = None
        self.map_info = None

        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            map_qos,
        )

        self.battery_percent = None
        self.battery_sub = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_callback,
            qos_profile_sensor_data,
        )

    def map_callback(self, msg: OccupancyGrid):
        with self.lock:
            self.map_data = np.array(msg.data, dtype=np.int8).reshape(
                (msg.info.height, msg.info.width)
            )
            self.map_info = msg.info

    def battery_callback(self, msg: BatteryState):
        with self.lock:
            self.battery_percent = msg.percentage

    def spin_thread(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()


def find_frontiers(grid):
    """Find frontier cells: free cells adjacent to at least one unknown cell.

    Returns a list of (row, col) tuples.
    """
    rows, cols = grid.shape
    frontier_cells = []

    # Pad to avoid boundary checks
    for r in range(1, rows - 1):
        for c in range(1, cols - 1):
            if grid[r, c] != FREE:
                continue
            # Check 4-connected neighbors for unknown
            if (grid[r - 1, c] == UNKNOWN or grid[r + 1, c] == UNKNOWN or
                    grid[r, c - 1] == UNKNOWN or grid[r, c + 1] == UNKNOWN):
                frontier_cells.append((r, c))

    return frontier_cells


def cluster_frontiers(frontier_cells, grid_shape):
    """Group frontier cells into connected clusters using BFS.

    Returns a list of clusters, each cluster being a list of (row, col).
    """
    if not frontier_cells:
        return []

    frontier_set = set(frontier_cells)
    visited = set()
    clusters = []

    # Walk every unvisited frontier cell and flood-fill its 8-connected neighbors into one cluster
    for cell in frontier_cells:
        if cell in visited:
            continue
        # BFS from this cell
        cluster = []
        queue = deque([cell])
        visited.add(cell)
        while queue:
            r, c = queue.popleft()
            cluster.append((r, c))
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nr, nc = r + dr, c + dc
                if (nr, nc) in frontier_set and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    queue.append((nr, nc))
        clusters.append(cluster)

    return clusters


def grid_to_world(row, col, map_info):
    """Convert grid coordinates to world (x, y) using the map metadata."""
    x = map_info.origin.position.x + (col + 0.5) * map_info.resolution
    y = map_info.origin.position.y + (row + 0.5) * map_info.resolution
    return x, y


def cluster_centroid_world(cluster, map_info):
    """Compute the world-frame centroid of a frontier cluster."""
    avg_r = sum(r for r, c in cluster) / len(cluster)
    avg_c = sum(c for r, c in cluster) / len(cluster)
    return grid_to_world(avg_r, avg_c, map_info)


def select_best_frontier(clusters, map_info, robot_x, robot_y):
    """Select the best frontier to explore.

    Uses a weighted score combining frontier size (more unknown to reveal)
    and proximity (closer is cheaper to reach).
    """
    best_score = -1
    best_centroid = None

    for cluster in clusters:
        if len(cluster) < MIN_FRONTIER_SIZE:
            continue

        cx, cy = cluster_centroid_world(cluster, map_info)
        dist = math.hypot(cx - robot_x, cy - robot_y)

        if dist < MIN_GOAL_DISTANCE:
            continue

        # Rank by size/distance ratio: prefer large, nearby frontiers
        size_score = len(cluster)
        dist_score = 1.0 / (dist + 0.1)
        score = size_score * dist_score

        if score > best_score:
            best_score = score
            best_centroid = (cx, cy)

    return best_centroid


def main(args=None):
    rclpy.init(args=args)

    lock = Lock()
    map_sub = MapSubscriber(lock)
    navigator = TurtleBot4Navigator()

    # Spin map subscriber in background thread
    thread = Thread(target=map_sub.spin_thread, daemon=True)
    thread.start()

    navigator.info('Frontier exploration starting')

    # Undock first (the docked robot has no lidar), localize, wait for Nav2
    undock_and_localize(navigator)
    navigator.info('Nav2 active — starting exploration')

    # Current robot position estimate (updated each iteration)
    robot_x, robot_y = 0.0, 0.0
    consecutive_no_frontier = 0

    while True:
        # --- Battery check ---
        with lock:
            battery_percent = map_sub.battery_percent

        if battery_percent is not None:
            navigator.info(f'Battery: {battery_percent * 100:.1f}%')

            if battery_percent < BATTERY_CRITICAL:
                navigator.error('Battery critically low — stopping exploration')
                break

            if battery_percent < BATTERY_LOW:
                navigator.info('Battery low — returning to dock')
                navigator.startToPose(
                    navigator.getPoseStamped([-1.0, 1.0], TurtleBot4Directions.EAST)
                )
                navigator.dock()
                if not navigator.getDockedStatus():
                    navigator.error('Failed to dock')
                    break

                navigator.info('Charging...')
                while battery_percent < BATTERY_HIGH:
                    sleep(15)
                    with lock:
                        battery_percent = map_sub.battery_percent
                    if battery_percent is None:
                        continue
                    navigator.info(f'Charging: {battery_percent * 100:.1f}%')

                navigator.undock()
                continue

        # --- Get map snapshot ---
        with lock:
            grid = map_sub.map_data
            info = map_sub.map_info

        if grid is None:
            navigator.info('Waiting for map data...')
            sleep(2)
            continue

        # --- Detect and cluster frontiers ---
        frontier_cells = find_frontiers(grid)
        clusters = cluster_frontiers(frontier_cells, grid.shape)

        navigator.info(
            f'Found {len(frontier_cells)} frontier cells in {len(clusters)} clusters'
        )

        # --- Select target frontier ---
        target = select_best_frontier(clusters, info, robot_x, robot_y)

        if target is None:
            consecutive_no_frontier += 1
            navigator.info(
                f'No valid frontiers found ({consecutive_no_frontier}/3)'
            )
            if consecutive_no_frontier >= 3:
                navigator.info('Exploration complete — no more frontiers')
                break
            sleep(3)
            continue

        consecutive_no_frontier = 0
        tx, ty = target
        navigator.info(f'Navigating to frontier at ({tx:.2f}, {ty:.2f})')

        # Build goal pose — orient the robot toward the frontier using atan2 for the yaw
        heading = math.atan2(ty - robot_y, tx - robot_x)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = navigator.get_clock().now().to_msg()
        goal.pose.position.x = tx
        goal.pose.position.y = ty
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(heading / 2.0)
        goal.pose.orientation.w = math.cos(heading / 2.0)

        navigator.startToPose(goal)

        # Update robot position estimate to the goal we just navigated to
        robot_x, robot_y = tx, ty

    # --- Done ---
    navigator.info('Frontier exploration finished — docking')
    navigator.startToPose(
        navigator.getPoseStamped([-1.0, 1.0], TurtleBot4Directions.EAST)
    )
    navigator.dock()

    map_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
