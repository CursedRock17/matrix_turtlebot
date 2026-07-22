# Navigate to a Goal
This is a simple set of instructions to navigate to a goal with a Matrix Turtlebot4

## Prerequisites
This tutorial assumes you've gone ahead and generate a map with SLAM, we now want to use
localization

Note: the robot publishes its topics (`/scan`, `/odom`, etc.) at the root namespace,
so no `namespace:=` argument is passed to any of the launches below. For a robot
configured with a namespace, pass `namespace:=<ns>` to each launch in steps 2–4 and
run the node in step 5 with `--ros-args -r __ns:=/<ns>`.

The robot should start **on its dock** — `nav_to_node` uses the dock as its known
starting pose.

The map you localize on must have a **locations file** next to it
(`maps/<name>.locations.yaml`, see `maps/robotics_lab.locations.yaml` for the
format): it holds that map's surveyed dock poses and named locations, and the
nav nodes refuse to start without it. The dock is only at `(0, 0)` on maps
whose SLAM run started on the dock, so these have to be surveyed per map.

### Steps:
We run the stack as four separate launches in four terminals. They are kept separate
on purpose: it's easier to see which layer failed, and starting them one at a time
(rather than all at once) avoids a discovery-registration burst on the robot's Pi
that can starve AMCL's heartbeat.

1) Open 4 Separate terminals, source each one of them for the project:
`source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash && source turtlebot4_bringup/setup.bash`
(sourcing `turtlebot4_bringup/setup.bash` also starts a local discovery server on the
laptop if one isn't running yet — see Troubleshooting for why that matters)

Before launching anything, sanity-check that the robot is actually on the wire —
this catches the "robot went silent after a power-cycle" case in seconds instead
of a 60 s nav2 abort, and drops a diagnostic bag in `claude_logs/`:
`ros2 run turtlebot4_custom_py preflight`
Wait for `VERDICT: robot is on the wire` before continuing. If it says NOT READY,
fix the `[FAIL]` lines (robot powered on / on BaleNet / `setup.bash` sourced) first.

2) In the First Terminal, run the visualization stack with RViz2:
`ros2 launch turtlebot4_viz view_navigation.launch.py`

3) In the Second Terminal, run the localization layer (from the workspace root so the
map path resolves):
`ros2 launch turtlebot4_bringup localization.launch.py map:=./maps/robotics_lab.yaml`
This is our own wrapper around `turtlebot4_navigation localization.launch.py` with the
localization lifecycle manager's bond watchdog disabled (it defaults `params` to
`turtlebot4_bringup/config/localization.config.yaml`). On large maps like `first_floor`,
map_server's executor gets blocked sending the big latched map and misses the 4 s bond
heartbeat, so the stock launch declares `CRITICAL FAILURE: SERVER map_server IS DOWN`
and kills localization ~4 s after it starts — see Troubleshooting.
Wait for the map to show up in RViz before moving on. Don't set the initial pose by
hand — the navigation node (step 5) reads it from the map's locations file. Also don't
worry that no laser scan is visible yet: the Turtlebot4 **stops the lidar while it is
docked**, and the scan only reappears once the robot undocks.

4) In the Third Terminal, open the navigation stack:
`ros2 launch turtlebot4_bringup nav2.launch.py`
This uses our own composed nav2 launch (single process) instead of
`turtlebot4_navigation nav2.launch.py` (11 processes), because the discovery burst of
11 nodes registering with the discovery server on the robot's Pi can starve AMCL's
bond heartbeat and make the localization lifecycle manager shut everything down
(`CRITICAL FAILURE: SERVER amcl IS DOWN`). The param file defaults to
`turtlebot4_bringup/config/nav2.config.yaml`.
**Don't wait for it to print `Managed nodes are active` — go straight to step 5.** On a
docked cold start nav2 *cannot* finish activating until the robot undocks: the global
costmap needs the `map` frame, which AMCL only publishes once it has laser scans
(robot undocked) and an initial pose. Until that line appears the `navigate_to_pose`
action does not exist and nothing can navigate (RViz's Nav2 Goal button included) —
but the navigation node waits for it itself, so just start it.

5) In the Fourth Terminal, run our Navigation node:
`ros2 run turtlebot4_custom_py nav_to_node`
It does, in order: dock (if not already docked) → **undock** (this restarts the
lidar) → set the initial pose to the spot just off the dock (read from the active
map's locations file) → wait for AMCL + bt_navigator → drive to the goal. The
undock-before-localize order is required: while docked there are no laser scans, AMCL
never publishes `amcl_pose`, and `waitUntilNav2Active()` would block forever re-sending
the initial pose.

Other navigation nodes (e.g. `nav_patrol_loop`) use the same steps 1–4 — only the node
in step 5 changes.

> There is also a `matrix_nav.launch.py` that bundles steps 2–4 into a single command,
> but the separate launches above are the recommended path: they're easier to debug and
> they stagger startup, which the combined launch does not.
