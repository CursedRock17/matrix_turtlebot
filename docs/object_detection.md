# Object Detection (YOLO26 + OAK-D Pro / webcam)

`yolo_detection` runs a YOLO model on a camera stream and publishes what it
sees. Inference runs on the **laptop** CPU (~17 FPS for yolo26n), not on the
OAK-D's VPU — that keeps the robot's camera pipeline untouched and lets the
same node consume the OAK-D topic, the laptop webcam, or a recorded bag.

## Setup (already done on this laptop)
- CPU-only torch: `pip install --user --break-system-packages torch torchvision --index-url https://download.pytorch.org/whl/cpu`
- `pip install --user --break-system-packages --no-deps ultralytics ultralytics-thop py-cpuinfo`
  (**`--no-deps` matters**: plain `pip install ultralytics` pulls in
  `opencv-python`, which shadows the system cv2 that `cv_bridge` is built
  against)
- Weights live in `models/yolo26n.pt` (git-tracked, 5.3 MB) so no internet is
  needed on BaleNet.

## Running
From the workspace root (so `models/` resolves):

```bash
# OAK-D Pro stream (default topic /oakd/rgb/preview/image_raw):
ros2 run turtlebot4_custom_py yolo_detection

# Laptop webcam, people only:
ros2 run turtlebot4_custom_py yolo_detection --ros-args -p webcam:=0 -p classes:=person
```

If nothing arrives from the OAK-D, check the actual topic name with
`ros2 topic list | grep oakd` and pass it via `-p image_topic:=...`.

## Topics
- `detections` (`vision_msgs/Detection2DArray`) — pixel-space bbox center +
  size, class name, confidence. This is what a person-follower consumes.
- `detections/image` (`sensor_msgs/Image`) — annotated frames; add an Image
  display in RViz to watch. Only rendered while someone is subscribed.

## Parameters
| param | default | meaning |
|---|---|---|
| `image_topic` | `/oakd/rgb/preview/image_raw` | ROS image source |
| `webcam` | `-1` | `/dev/videoN` index; `>= 0` overrides the topic |
| `model` | `models/yolo26n.pt` | any ultralytics-loadable weights |
| `conf` | `0.4` | confidence threshold |
| `rate` | `10.0` | max inference Hz (stale frames are dropped) |
| `classes` | `''` (all) | comma-separated filter, e.g. `person,dog` |

## Toward following people
The pieces a follow behavior needs from here:
1. Bearing to each person: bbox center x vs. image width + the camera's
   horizontal FOV (from `camera_info`).
2. Distance: read the OAK-D's stereo depth topic at the bbox center.
3. A controller that keeps the group centered/at range — either direct
   `cmd_vel` (simple, but bypasses Nav2's obstacle avoidance) or periodic
   `navigate_to_pose` goals toward the group centroid (safer, reuses Nav2).

Streaming note: the RGB preview over BaleNet plus discovery traffic is
non-trivial Wi-Fi load. If the stream lags, lower the camera FPS on the Pi or
keep `rate` modest — the node always processes the newest frame only.
