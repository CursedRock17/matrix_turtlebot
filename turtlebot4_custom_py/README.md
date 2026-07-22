# turtlebot4_custom_py

The lab's Python examples for the TurtleBot4. Each node is one self-contained
example; the libraries at the bottom hold the startup and map logic they share.

## What's in here

| File | What it does |
|------|--------------|
| `nav_to_node.py` | Undocks, localizes from the map's known dock pose, and drives to a single hardcoded goal — the smallest end-to-end navigation example. |
| `nav_patrol_loop.py` | Patrols the active map's waypoint list forever, docking itself to recharge when the battery runs low. |
| `command_control.py` | Same patrol-and-recharge loop as `nav_patrol_loop`, but with waypoints hardcoded in the file instead of read from the map's locations file. |
| `llm_navigation_node.py` | Listens for natural-language commands on a topic and uses a local LLM to turn them into navigation goals. |
| `patrol_with_llm_node.py` | Patrols like `nav_patrol_loop`, but accepts LLM reroute commands mid-patrol and then resumes. |
| `yolo_detection_node.py` | Runs a YOLO model on the camera stream (or a webcam) and publishes the detections, plus annotated frames for RViz. |
| `frontier_exploration.py` | Frontier-based autonomous exploration of an unknown map — work in progress, no entry point in `setup.py` yet. |
| `preflight.py` | Pre-launch health check ("doctor"): confirms the robot is actually publishing (`/odom`, `/battery_state`, `/scan`), pings it, checks the discovery server, and records a diagnostic bag — run it before launching to catch a silent robot in seconds instead of a 60 s nav2 abort. Exits non-zero if the robot isn't on the wire. |
| `wifi_survey.py` | Wi-Fi site-survey logger: samples signal/BSSID/bitrate every few seconds, pairs each sample with the AMCL pose, and appends CSV — plot it over the map to find dead zones and AP-roam spots. Deliberately standalone (rclpy only) so it can be `scp`'d to and run **on the Pi**, whose radio is the one that matters. |
| `bump_to_cloud.py` | Turns Create 3 bumper hits into short-lived costmap obstacles on `bump_points`: feet are invisible to the lidar, so after the firmware's bump reflex this is what makes nav2 actually route around whatever it hit instead of retrying the same path. Run on the laptop alongside the nav stack. |
| `survey_locations.py` | Surveying helper: type a name, click the spot in RViz with 'Publish Point', and it prints a ready-to-paste `locations:` block for the active map's locations file. |
| `merge_maps.py` | Offline CLI that merges two saved maps into one larger map, given the transform between their frames (see [docs/generate_a_map.md](../docs/generate_a_map.md)). |
| `location_mapper_eval.py` | Scores the LLM location mapper against a fixed prompt set, including must-abstain cases — run it after changing the model, prompt, or locations; no robot needed. |
| `llm_location_mapper.py` | Library: prompts a local GGUF LLM to extract a known location from a sentence; also runnable standalone (`location_mapper`) to test the LLM without a robot. |
| `map_locations.py` | Library: loads the surveyed dock poses and named locations for whatever map the running `map_server` has loaded (`maps/<name>.locations.yaml`). |
| `startup.py` | Library: the shared dock → undock → localize → wait-for-Nav2 sequence every navigation example starts with. Also restarts the lidar if it didn't come back after undocking. |
| `monitors.py` | Library: the background monitor nodes the loops spin in daemon threads — `BatteryMonitor` (latest charge percentage) and `ScanWatchdog` (is `/scan` fresh AND time-synced, i.e. is nav2 actually seeing the lidar). |
| `locations_map.txt` | The fallback name → pose list for the LLM location mapper — used only when no map is active; with a map loaded the names come from its locations file. |
| `first_floor_locations.txt` | Raw survey notes from the first-floor mapping runs — not loaded by any code. The live poses are in `maps/first_floor.locations.yaml`; this keeps a few landmarks that were never copied over. |

Every node runs as `ros2 run turtlebot4_custom_py <entry point>`. Entry points
match the file names except `llm_navigation_node.py` → `llm_navigation`,
`patrol_with_llm_node.py` → `patrol_with_llm`,
`yolo_detection_node.py` → `yolo_detection`, and
`llm_location_mapper.py` → `location_mapper` (full list in `setup.py`).

## Where to start: crawl, walk, run

Every example assumes the robot stack is already up: a generated map with a
locations file beside it, the robot on its dock, and the laptop-side launch
running — follow [docs/navigate_to_a_goal.md](../docs/navigate_to_a_goal.md)
first. Then work through the examples in order; each one adds a single new
idea on top of the last, so when something breaks you know which layer broke.

- **Crawl — `nav_to_node`.** One trip: undock, localize, drive to a goal.
  If this works, your bringup, time sync, discovery, and map locations file
  are all good. Nothing later works until this does.
- **Walk — `nav_patrol_loop`.** The same startup, now in a loop with battery
  management: patrol the map's waypoints, dock to charge, resume.
- **Run — `llm_navigation`.** Natural language in, navigation goals out.
  Try `location_mapper` by itself first to test the LLM with no robot at all,
  then `location_mapper_eval` to score it against the fixed prompt set
  (see [LLM_NAVIGATION_README.md](./LLM_NAVIGATION_README.md)).
- **Integrate.** `patrol_with_llm` composes the patrol loop with LLM
  rerouting; `yolo_detection` adds perception alongside whatever is driving
  (see [docs/object_detection.md](../docs/object_detection.md)). New research
  builds at this layer, out of the pieces you just proved one by one.
