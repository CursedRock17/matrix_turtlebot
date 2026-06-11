# Patrol Loop

Continuously cycle through a set of waypoints with battery monitoring.
The robot auto-docks when battery drops below 30% and resumes patrol once
charged above 95%.

## Prerequisites
- A saved map (see [Generate a Map](../generate_a_map.md))
- Nav2 and localization packages installed

## Steps

Open **4 terminals**. Source and build each one:
```bash
source /opt/ros/jazzy/setup.bash 
colcon build 
source install/setup.bash 
source turtlebot4_bringup/setup.bash
```
You should also add a namespace if you have one:
```bash
export NAMESPACE="namespace"
```

### Terminal 1 - RViz2
Launch the Visualizer, RVIZ 2 window
```bash
ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=/$NAMESPACE
```

### Terminal 2 - Localization
We can now localize with our desired map
```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/$NAMESPACE map:=./maps/some_map.yaml
```

### Terminal 3 - Nav2
Wait for the robot to localize in RViz, set the initial pose, then:
```bash
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/$NAMESPACE
```

### Terminal 4 - Run the node
```bash
ros2 run turtlebot4_custom_py nav_patrol_loop
```

## What it does
1. Docks and sets initial pose
2. Waits for Nav2, then undocks
3. Loops through 3 waypoints indefinitely
4. Monitors `battery_state` in a background thread
5. When battery < 30%: navigates near the dock, docks, and waits for 95% charge
6. When battery < 12%: logs a critical error and shuts down : According to [official git](https://github.com/turtlebot/turtlebot4/blob/7fd29fb420e906f3aca4a904adb54b69b11c7c00/turtlebot4_node/src/turtlebot4.cpp#L351)

## Customizing waypoints
Edit the `goal_pose` list in `turtlebot4_custom_py/turtlebot4_custom_py/nav_patrol_loop.py`,
then rebuild.

## Alternative: command_control
The `command_control` node is a similar patrol demo with 4 waypoints:
```bash
ros2 run turtlebot4_custom_py command_control
```
