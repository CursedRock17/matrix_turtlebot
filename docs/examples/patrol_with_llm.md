# Patrol with LLM Rerouting

The robot continuously patrols a set of waypoints but can be dynamically
rerouted via natural language commands. After completing the reroute, it
resumes the patrol loop. Includes battery management.

## Prerequisites
- A saved map (see [Generate a Map](../generate_a_map.md))
- Nav2 and localization packages installed
- GGUF model downloaded (see [LLM Integration](../llm_integration.md))
- Python dependency: `pip install llama-cpp-python`

## Steps

Open **5 terminals**. Source each one:
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
Set the initial pose in RViz, then:
```bash
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/matrix_turtlebot1
```

### Terminal 4 - Patrol + LLM Node
```bash
ros2 run turtlebot4_custom_py patrol_with_llm
```
The robot will undock and begin patrolling 3 waypoints.

### Terminal 5 - Send reroute commands
At any time, interrupt the patrol with a natural language command:
```bash
ros2 topic pub --once /navigation_command std_msgs/String "data: 'Take this to Harold'"
```
The robot will finish its current waypoint, reroute to Harold's Room, then
resume the patrol from where it left off.

## What it does
1. Starts the LLM mapper and patrol node in background threads
2. Docks, sets initial pose, waits for Nav2, undocks
3. Patrols 3 waypoints in a loop
4. Between each waypoint, checks for commands on `/navigation_command`
5. If a command is queued: extracts the location via LLM, navigates there,
   then resumes patrol
6. Battery management: auto-docks at 30%, resumes at 95%, shuts down at 10%

## Customizing patrol waypoints
Edit the `goal_pose` list in
`turtlebot4_custom_py/turtlebot4_custom_py/patrol_with_llm_node.py`,
then rebuild.
