# Frontier Exploration

Autonomously explore and build a map of unknown space. The robot detects
frontiers (boundaries between known free space and the unknown), picks the
best one, navigates there, and repeats until the environment is fully mapped.

## Prerequisites
- Nav2 installed (`sudo apt install ros-jazzy-turtlebot4-navigation`)
- **No pre-built map needed** - this demo builds the map from scratch using SLAM

## Steps

Open **4 terminals**. Source each one:
```bash
source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash && source turtlebot4_bringup/setup.bash
```

### Terminal 1 - RViz2
```bash
ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=/matrix_turtlebot1
```

### Terminal 2 - SLAM (not localization)
Since we are building a new map, we use synchronous SLAM instead of localization:
```bash
ros2 launch turtlebot4_navigation slam.launch.py sync:=true params:=./turtlebot4_bringup/config/slam.config.yaml namespace:=matrix_turtlebot1
```

### Terminal 3 - Nav2
```bash
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/matrix_turtlebot1
```

### Terminal 4 - Run the node
```bash
ros2 run turtlebot4_custom_py frontier_exploration
```

## What it does
1. Docks and sets initial pose at the dock
2. Waits for Nav2, then undocks
3. Subscribes to `/map` (OccupancyGrid published by SLAM)
4. Each iteration:
   - Scans the map for frontier cells (free cells next to unknown cells)
   - Clusters frontiers into groups using BFS
   - Scores each cluster by `size / distance` — prefers large, nearby frontiers
   - Navigates to the best frontier's centroid
5. Stops after 3 consecutive iterations with no valid frontiers (map is complete)
6. Docks when finished

Battery management is included — the robot will dock and charge if battery
drops below 30%.

## Saving the map
Once exploration is done, save the map from another sourced terminal:
```bash
ros2 run nav2_map_server map_saver_cli -f "./maps/map_name" --ros-args -p map_subscribe_transient_local:=true -r __ns:=/matrix_turtlebot1
```

## Tuning parameters
These constants at the top of `frontier_exploration.py` control behavior:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MIN_FRONTIER_SIZE` | 5 | Minimum cells in a cluster to consider |
| `MIN_GOAL_DISTANCE` | 0.3 m | Ignore frontiers closer than this |
| `EXPLORATION_TIMEOUT` | 120 s | Max time to reach a frontier before retrying |
