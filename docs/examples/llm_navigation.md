# LLM Navigation

Send natural language commands to the robot and let a local LLM figure out
where to go. Supports both single-destination and multi-destination (queue)
commands.

## Prerequisites
- A saved map (see [Generate a Map](../generate_a_map.md))
- Nav2 and localization packages installed
- GGUF model downloaded (see [LLM Integration](../llm_integration.md))
  ```bash
  # Model should be at:
  # ~/Documents/Electrical/Matrix_Lab/turtlebot4_ws/models/Llama-3.2-3B-Instruct-Q4_K_M.gguf
  ```
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

### Terminal 4 - LLM Navigation Node
```bash
ros2 run turtlebot4_custom_py llm_navigation
```
Wait for `LLM Navigation Node is ready!` before sending commands.

### Terminal 5 - Send commands
Single destination:
```bash
ros2 topic pub --once /navigation_command std_msgs/String "data: 'Bring Harold this book'"
```

Multi-destination queue:
```bash
ros2 topic pub --once /navigation_command std_msgs/String "data: 'Go to Harold'\''s room, then John'\''s room, then back to the dock'"
```

The robot will visit each location in order and log progress like
`[1/3] Navigating to...`, `[2/3] Navigating to...`, etc.

## How it works
1. A command string arrives on the `/navigation_command` topic
2. The LLM extracts all mentioned locations in order (comma-separated)
3. Each location is fuzzy-matched against `locations_map.txt`
4. The robot navigates to each waypoint sequentially

## Adding locations
Edit `turtlebot4_custom_py/turtlebot4_custom_py/locations_map.txt`:
```
Harold's Room: 0.0, 1.0, NORTH
John's Room: -1.0, 3.0, NORTH
Kitchen: 2.0, -1.5, EAST
dock: 0.0, 0.0, NORTH
```

Rebuild after changes: `colcon build --packages-select turtlebot4_custom_py`

## Testing the LLM standalone
You can verify location extraction without the robot:
```bash
cd turtlebot4_custom_py/turtlebot4_custom_py
python3 llm_location_mapper.py
```

## Parameters
```bash
ros2 run turtlebot4_custom_py llm_navigation \
  --ros-args \
  -p robot_namespace:=/matrix_turtlebot1 \
  -p model_path:=/path/to/model.gguf \
  -p command_topic:=navigation_command \
  -p n_threads:=4
```

## Example commands
| Command | Result |
|---------|--------|
| "Bring Harold this book" | Harold's Room |
| "Go to John's room" | John's Room |
| "Go back to the dock" | dock |
| "Visit Harold, then John, then dock" | Harold's Room -> John's Room -> dock |
