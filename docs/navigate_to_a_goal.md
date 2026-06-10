# Navigate to a Goal
This is a simple set of instructions to navigate to a goal with a Matrix Turtlebot4

## Prerequisites
This tutorial assumes you've gone ahead and generate a map with SLAM, we now want to use
localization

Note: the robot publishes its topics (`/scan`, `/odom`, etc.) at the root namespace,
so no `namespace:=` argument is passed to any of the launches below.

The robot should start **on its dock** — `nav_to_node` uses the dock as its known
starting pose.

### Steps:
1) Open 4 Separate terminals, source each one of them for the project:
`source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash && source turtlebot4_bringup/setup.bash`
(sourcing `turtlebot4_bringup/setup.bash` also starts a local discovery server on the
laptop if one isn't running yet — see Troubleshooting for why that matters)
2) In the First Terminal, we want to run our visualization stack with RViz2:
`ros2 launch turtlebot4_viz view_navigation.launch.py`
3) In the Second Terminal, we want to run the localization layer with our param file:
`ros2 launch turtlebot4_navigation localization.launch.py map:=./maps/mini_paa.yaml params:=./turtlebot4_bringup/config/localization.config.yaml`
4) Wait for the map to show up in RViz2. Don't worry about setting the initial pose by
hand — `nav_to_node` (step 6) sets it itself from the known dock position. Also don't
worry that no laser scan is visible yet: the Turtlebot4 **stops the lidar while it is
docked**, and the scan only reappears once the robot undocks.
5) Open the navigation stack. This uses our own composed nav2 launch (single process)
instead of `turtlebot4_navigation nav2.launch.py` (11 processes), because the
discovery burst of 11 nodes registering with the discovery server on the
robot's Pi can starve AMCL's bond heartbeat and make the localization
lifecycle manager shut everything down
(`CRITICAL FAILURE: SERVER amcl IS DOWN`). The param file defaults to
`turtlebot4_bringup/config/nav2.config.yaml`:
`ros2 launch turtlebot4_bringup nav2.launch.py`
Wait until it prints **`Managed nodes are active`** — until that line appears the
`navigate_to_pose` action does not exist and nothing can navigate (RViz's Nav2 Goal
button included).
6) Run our Navigation node:
`ros2 run turtlebot4_custom_py nav_to_node`
It does, in order: dock (if not already docked) → **undock** (this restarts the
lidar) → set the initial pose to the spot just off the dock → wait for AMCL +
bt_navigator → drive to the goal. The undock-before-localize order is required:
while docked there are no laser scans, AMCL never publishes `amcl_pose`, and
`waitUntilNav2Active()` would block forever re-sending the initial pose.

### Troubleshooting
- **Step 5 never prints `Managed nodes are active` / stuck after `Activating bt_navigator`**:
  bt_navigator's activation builds the behavior tree, whose action clients must be
  matched with their servers by a discovery server. If the only discovery server is the
  one on the robot's Pi, that matching crosses the Wi-Fi and can stall forever. The fix
  is the local discovery server on the laptop (`127.0.0.1:11888`), which
  `turtlebot4_bringup/setup.bash` starts automatically. Check it's alive with
  `ss -lun | grep 11888` (you should see a UDP listener), re-source the
  setup.bash if not, then restart the nav2 launch.
- **RViz says `navigate_to_pose action server is not available`**: same cause as above —
  bt_navigator isn't active yet (or never became active).
- **`nav_to_node` loops printing `Waiting for amcl_pose to be received`**: AMCL isn't
  getting laser scans. Check `ros2 topic hz /scan`; if the robot is docked the lidar is
  off — that's the bug the current `nav_to_node` ordering avoids, so make sure you're
  running the rebuilt version (`colcon build` + re-source).
- **Robot disappears from RViz when starting nav2 / `CRITICAL FAILURE: SERVER amcl IS DOWN`**:
  the localization lifecycle manager stopped receiving AMCL's heartbeat (4 s timeout)
  because discovery/CPU got overloaded. Make sure you are using the composed launch in
  step 5, check laptop CPU with `htop`, and check the Wi-Fi link to the robot with
  `ping 192.168.50.223` while nav2 starts.
- **Nav2 hangs at "Waiting for service ... get_state"**: discovery traffic is not
  flowing. Verify the discovery server on the robot is reachable (`ping 192.168.50.223`)
  and that stale ROS processes from earlier runs are killed.
- **Robot drives but localization drifts or jumps**: run the time-sync pre-flight check
  (`ros2 topic delay /odom`, want ~0.01–0.05 s) — see [time_sync.md](./time_sync.md).
