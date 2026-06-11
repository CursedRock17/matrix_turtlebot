# Navigate to a Goal

Drive the robot to a single goal pose and stop.

## Prerequisites
- A saved map (see [Generate a Map](../generate_a_map.md))
- Nav2 and localization packages installed (`sudo apt install ros-jazzy-turtlebot4-navigation`)

## Steps

Open **4 terminals**. Source each one:
```bash
source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash && source turtlebot4_bringup/setup.bash
```

### Terminal 1 - RViz2
```bash
ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=/matrix_turtlebot1
```

### Terminal 2 - Localization
```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/matrix_turtlebot1 map:=./maps/lucas_room.yaml
```

### Terminal 3 - Nav2
Wait for the robot to appear in RViz, then set its initial pose using the
**2D Pose Estimate** button. Once the laser scan aligns with the map, launch Nav2:
```bash
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/matrix_turtlebot1
```

### Terminal 4 - Run the node
Once the costmap is visible in RViz:
```bash
ros2 run turtlebot4_custom_py nav_to_node
```

## What it does
1. Docks (if not already docked) and sets the initial pose at the dock
2. Waits for Nav2 to be fully active
3. Undocks and navigates to the goal at `[1.0, 2.0]` facing EAST
4. Shuts down

## Customizing the goal
Edit `turtlebot4_custom_py/turtlebot4_custom_py/nav_to_node.py` and change the
coordinates in the `getPoseStamped` call, then rebuild with `colcon build`.
