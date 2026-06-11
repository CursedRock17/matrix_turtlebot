# Examples

Step-by-step guides for running each demo on the MATRIX Lab TurtleBot4s.

Every example assumes you have already completed the [setup guide](../setup.md) and
have a saved map (see [Generate a Map](../generate_a_map.md)) unless otherwise noted.

## Common Setup

Each demo requires sourcing the workspace. In **every** terminal you open, run:
```bash
source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash && source turtlebot4_bringup/setup.bash
```

## Demos

| Demo | Description | Guide |
|------|-------------|-------|
| Navigate to a Goal | Drive to a single waypoint and stop | [nav_to_goal.md](nav_to_goal.md) |
| Patrol Loop | Cycle through waypoints with battery management | [patrol_loop.md](patrol_loop.md) |
| Frontier Exploration | Autonomously explore and map unknown space | [frontier_exploration.md](frontier_exploration.md) |
| LLM Navigation | Natural language commands (single or multi-destination) | [llm_navigation.md](llm_navigation.md) |
| Patrol with LLM | Patrol loop with live LLM rerouting | [patrol_with_llm.md](patrol_with_llm.md) |
| Object Detection | YOLOv8 + depth + SLAM map projection | [object_detection.md](object_detection.md) |


