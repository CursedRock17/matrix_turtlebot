# Navigate to a Goal
This is a simple set of instructions to navigate to a goal with a Matrix Turtlebot4

## Prerequisites
This tutorial assumes you've gone ahead and generate a map with SLAM, we now want to use
localization

Note: the robot publishes its topics (`/scan`, `/odom`, etc.) at the root namespace,
so no `namespace:=` argument is passed to any of the launches below. For a robot
configured with a namespace, pass `namespace:=<ns>` to the launch in step 2 and
run the node in step 3 with `--ros-args -r __ns:=/<ns>`.

The robot should start **on its dock** — `nav_to_node` uses the dock as its known
starting pose.

The map you localize on must have a **locations file** next to it
(`maps/<name>.locations.yaml`, see `maps/robotics_lab.locations.yaml` for the
format): it holds that map's surveyed dock poses and named locations, and the
nav nodes refuse to start without it. The dock is only at `(0, 0)` on maps
whose SLAM run started on the dock, so these have to be surveyed per map.

### Steps:
1) Open 2 Separate terminals, source each one of them for the project:
`source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash && source turtlebot4_bringup/setup.bash`
(sourcing `turtlebot4_bringup/setup.bash` also starts a local discovery server on the
laptop if one isn't running yet — see Troubleshooting for why that matters)
2) In the First Terminal, start the whole laptop-side stack — RViz, localization,
and nav2 — with one launch (run it from the workspace root so the map paths resolve):
`ros2 launch turtlebot4_bringup matrix_nav.launch.py map:=./maps/robotics_lab.yaml`
Pass `use_rviz:=false` to skip RViz; the three layers can still be launched
individually (`turtlebot4_viz view_navigation.launch.py`,
`turtlebot4_navigation localization.launch.py`, `turtlebot4_bringup nav2.launch.py`)
when debugging one of them in isolation.
Notes on what this runs:
   - nav2 uses our own composed launch (single process) instead of
     `turtlebot4_navigation nav2.launch.py` (11 processes), because the discovery
     burst of 11 nodes registering with the discovery server on the robot's Pi can
     starve AMCL's bond heartbeat and make the localization lifecycle manager shut
     everything down (`CRITICAL FAILURE: SERVER amcl IS DOWN`).
   - Wait for the map to show up in RViz, but don't set the initial pose by hand —
     the navigation node (step 3) reads it from the map's locations file. Don't
     worry that no laser scan is visible yet: the Turtlebot4 **stops the lidar
     while it is docked**, and the scan only reappears once the robot undocks.
   - Don't wait for nav2 either — go straight to step 3. On a docked cold start
     nav2 *cannot* finish activating until the robot undocks: the global costmap
     needs the `map` frame, which AMCL only publishes once it has laser scans
     (robot undocked) and an initial pose. Once that happens nav2 prints
     **`Managed nodes are active`**; until that line appears the
     `navigate_to_pose` action does not exist and nothing can navigate (RViz's
     Nav2 Goal button included) — but the navigation node waits for it itself.
3) In the Second Terminal, run our Navigation node:
`ros2 run turtlebot4_custom_py nav_to_node`
It does, in order: dock (if not already docked) → **undock** (this restarts the
lidar) → set the initial pose to the spot just off the dock (read from the
active map's locations file) → wait for AMCL + bt_navigator → drive to the goal. The undock-before-localize order is required:
while docked there are no laser scans, AMCL never publishes `amcl_pose`, and
`waitUntilNav2Active()` would block forever re-sending the initial pose.
