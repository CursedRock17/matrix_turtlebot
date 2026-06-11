# Object Detection with SLAM

Run YOLOv8 on the Oak-D camera feed, project detections into the SLAM map
via depth + TF, and visualize them as floating labels in RViz.

## Prerequisites
- Nav2 and localization packages installed
- A saved map, or use SLAM to build one live
- Python dependency: `pip install ultralytics`
- The YOLOv8 model downloads automatically on first run (~6 MB for `yolov8n.pt`)

## Steps

Open **5 terminals**. Source each one:
```bash
source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash && source turtlebot4_bringup/setup.bash
```

### Terminal 1 - RViz2
```bash
ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=/matrix_turtlebot1
```
In RViz, add a **MarkerArray** display and set its topic to
`/detected_objects_markers` to see detection labels on the map.

### Terminal 2 - Localization (or SLAM)
With an existing map:
```bash
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/matrix_turtlebot1 map:=./maps/lucas_room.yaml
```
Or with live SLAM (no map needed):
```bash
ros2 launch turtlebot4_navigation slam.launch.py sync:=true params:=./turtlebot4_bringup/config/slam.config.yaml namespace:=matrix_turtlebot1
```

### Terminal 3 - Nav2
Set the initial pose in RViz (if using localization), then:
```bash
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/matrix_turtlebot1
```

### Terminal 4 - Object Detection Node
```bash
ros2 run turtlebot4_custom_py object_detection
```
Wait for `Object Detection Node is ready!` in the log.

### Terminal 5 - Drive the robot (optional)
Teleop to move the robot around and detect objects in different areas:
```bash
ros2 launch turtlebot4_bringup joy_teleop.launch.py namespace:=matrix_turtlebot1
```
Or run any of the autonomous navigation nodes (patrol, frontier, etc.)
alongside the detection node.

## What it does
1. Subscribes to synchronized RGB + depth images from the Oak-D
2. Runs YOLOv8 inference on each RGB frame (~5 FPS by default)
3. For each detection:
   - Reads the depth at the bounding box center
   - Uses camera intrinsics to deproject the pixel into a 3D point
   - Transforms the point from `oakd_rgb_camera_optical_frame` into the `map` frame via TF
4. Publishes:
   - `/detected_objects_markers` (MarkerArray) — green floating labels in RViz
   - `/detected_objects` (String) — JSON array for downstream nodes

## Published JSON format
Each message on `/detected_objects` is a JSON array:
```json
[
  {
    "label": "person",
    "confidence": 0.87,
    "depth_m": 2.31,
    "map_x": 1.45,
    "map_y": 0.82,
    "map_z": 0.65
  }
]
```

## Parameters
```bash
ros2 run turtlebot4_custom_py object_detection \
  --ros-args \
  -p model:=yolov8n.pt \
  -p confidence_threshold:=0.5 \
  -p image_topic:=oakd/rgb/preview/image_raw \
  -p depth_topic:=oakd/rgb/preview/depth \
  -p camera_info_topic:=oakd/rgb/preview/camera_info \
  -p camera_frame:=oakd_rgb_camera_optical_frame \
  -p detection_interval:=0.2
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model` | `yolov8n.pt` | YOLO model (n/s/m/l/x for speed vs accuracy) |
| `confidence_threshold` | `0.5` | Minimum confidence to publish a detection |
| `detection_interval` | `0.2` | Seconds between inference runs (0.2 = ~5 FPS) |
| `camera_frame` | `oakd_rgb_camera_optical_frame` | TF frame for deprojection |

## Combining with other demos
The detection node runs independently — you can launch it alongside any
navigation demo. For example, run frontier exploration while detecting objects:

**Terminal 4:** `ros2 run turtlebot4_custom_py frontier_exploration`
**Terminal 5:** `ros2 run turtlebot4_custom_py object_detection`

The robot explores autonomously while labeling everything it sees on the map.

## Notes on performance
- **yolov8n** (nano) is fastest, good for real-time on CPU
- **yolov8s/m** are more accurate but slower
- If the LLM node is also running and using the GPU, consider setting
  `device=cpu` in ultralytics or reducing the detection interval
- The Oak-D's built-in Myriad X VPU can run YOLO natively via `depthai` —
  this is a future optimization that offloads the host entirely
