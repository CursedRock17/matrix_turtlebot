# LLM-Based Navigation for TurtleBot4

This package provides natural language navigation capabilities for the TurtleBot4 using LLAMA3.1 for context mapping.

## Overview

The system allows you to send natural language commands like "Bring Harold this book" and the robot will automatically navigate to Harold's Room using the LLAMA3.1 model to extract the location from the command.

## Files Created

1. **locations_map.txt** - Configuration file containing location mappings
   - Format: `Location Name: x, y, DIRECTION`
   - Example: `Harold's Room: 0.0, 1.0, NORTH`

2. **llm_location_mapper.py** - Core LLM integration module
   - Loads and initializes LLAMA3.1 model
   - Extracts locations from natural language commands
   - Maps locations to coordinates

3. **llm_navigation_node.py** - ROS2 node for navigation
   - Subscribes to command topic
   - Uses LLM to extract location
   - Commands TurtleBot4Navigator to navigate

## Installation

### 1. Install Python Dependencies

On the computer running the LLM (can be separate from the robot):

```bash
cd /home/cursedrock17/Documents/Electrical/Matrix_Lab/turtlebot4_ws/turtlebot4_custom_py
pip install -r requirements.txt
```

### 2. Setup HuggingFace Authentication (for LLAMA3.1)

You need to authenticate with HuggingFace to download LLAMA3.1:

```bash
# Install huggingface-cli if not already installed
pip install huggingface-hub

# Login to HuggingFace (you'll need an account and accept Meta's license)
huggingface-cli login
```

Then visit: https://huggingface.co/meta-llama/Meta-Llama-3.1-8B-Instruct
- Accept the license agreement for LLAMA3.1

### 3. Build the ROS2 Package

```bash
cd /home/cursedrock17/Documents/Electrical/Matrix_Lab/turtlebot4_ws
colcon build --packages-select turtlebot4_custom_py
source install/setup.bash
```

## Usage

### Running the Navigation Node

```bash
# Source your workspace
source /home/cursedrock17/Documents/Electrical/Matrix_Lab/turtlebot4_ws/install/setup.bash

# Run the LLM navigation node
ros2 run turtlebot4_custom_py llm_navigation
```

### Sending Commands

From another terminal, publish natural language commands:

```bash
# Example: Navigate to Harold's Room
ros2 topic pub /navigation_command std_msgs/msg/String "data: 'Bring Harold this book'"

# Example: Navigate to John's Room
ros2 topic pub /navigation_command std_msgs/msg/String "data: 'Go to John'"

# Example: Return to dock
ros2 topic pub /navigation_command std_msgs/msg/String "data: 'Go back to the charging station'"
```

### Parameters

You can customize the node with parameters:

```bash
ros2 run turtlebot4_custom_py llm_navigation \
  --ros-args \
  -p model_name:=meta-llama/Meta-Llama-3.1-8B-Instruct \
  -p command_topic:=navigation_command
```

For a namespaced robot, namespace the whole node with the standard remap
(this moves the navigator *and* the command topic together):

```bash
ros2 run turtlebot4_custom_py llm_navigation --ros-args -r __ns:=/matrix_turtlebot1
```

## Adding New Locations

Edit `turtlebot4_custom_py/locations_map.txt`:

```
Harold's Room: 0.0, 1.0, NORTH
John's Room: -1.0, 3.0, NORTH
Kitchen: 2.0, -1.5, EAST
Living Room: 5.0, 3.0, WEST
dock: 0.0, 0.0, NORTH
```

Available directions:
- NORTH, SOUTH, EAST, WEST
- NORTH_EAST, NORTH_WEST, SOUTH_EAST, SOUTH_WEST

## Testing the LLM Mapper Standalone

You can test the LLM location extraction without running the full ROS2 node:

```bash
cd /home/cursedrock17/Documents/Electrical/Matrix_Lab/turtlebot4_ws/turtlebot4_custom_py/turtlebot4_custom_py
python3 llm_location_mapper.py
```

This will run a series of test commands and show how the LLM extracts locations.

## System Architecture

```
┌─────────────────────┐
│  Natural Language   │
│     Command         │
└──────────┬──────────┘
           │
           ↓
┌─────────────────────┐
│  ROS2 Topic         │
│ /navigation_command │
└──────────┬──────────┘
           │
           ↓
┌─────────────────────┐
│ llm_navigation_node │
└──────────┬──────────┘
           │
           ↓
┌─────────────────────┐
│ llm_location_mapper │
│   (LLAMA3.1)        │
└──────────┬──────────┘
           │
           ↓ (x, y, direction)
┌─────────────────────┐
│ TurtleBot4Navigator │
└─────────────────────┘
```

## Hardware Requirements

### Recommended Specs for Running LLAMA3.1-8B:
- **GPU**: NVIDIA GPU with 16GB+ VRAM (recommended)
  - RTX 3090, RTX 4090, A5000, etc.
- **RAM**: 32GB+ system RAM
- **Storage**: 20GB for model weights

### Running on Separate Computer:
Since you mentioned running on another computer via ROS2, ensure:
1. Both computers are on the same ROS_DOMAIN_ID
2. Network connectivity between machines
3. ROS2 topics are properly forwarded

## Troubleshooting

### Model Download Issues
- Ensure you're authenticated with HuggingFace
- Check internet connection
- Verify you've accepted the LLAMA3.1 license

### Out of Memory
- Try using quantized versions (4-bit or 8-bit)
- Reduce batch size or context length
- Use CPU inference (slower but uses less VRAM)

### Location Not Found
- Check that location names in commands match those in locations_map.txt
- The LLM uses fuzzy matching, so exact names aren't required
- Check node logs for extracted location names

## Example Commands That Work

- "Bring Harold this book" → Harold's Room
- "Go to John's room" → John's Room
- "Take this to Harold" → Harold's Room
- "Navigate to John" → John's Room
- "Go back to the dock" → dock
- "Return to charging station" → dock

## Notes

- **No Training Required**: LLAMA3.1 is used for inference only (zero-shot)
- The model learns from the prompt which includes available locations
- First run will download ~16GB of model weights
- Subsequent runs will use cached model from ~/.cache/huggingface/

## Future Enhancements

Potential improvements:
- Add intent extraction (deliver, patrol, return, etc.)
- Multi-step navigation with waypoints
- Voice command integration
- Object recognition for "bring the book" style commands
- Dynamic location updates via service calls
