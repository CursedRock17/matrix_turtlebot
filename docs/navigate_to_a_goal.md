# Navigate to a Goal
This is a simple set of instructions to navigate to a goal with a Matrix Turtlebot4

## Prerequisites
This tutorial assumes you've gone ahead and generate a map with SLAM, we now want to use 
localization

Note: the robot publishes its topics (`/scan`, `/odom`, etc.) at the root namespace,
so no `namespace:=` argument is passed to any of the launches below.

### Steps:
1) Open 4 Separate terminals, source each one of them for the project:
`source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash && source turtlebot4_bringup/setup.bash`
2) In the First Terminal, we want to run our visualization stack with RViz2:
`ros2 launch turtlebot4_viz view_navigation.launch.py`
3) In the Second Terminal, we want to run the localization layer with our param file:
`ros2 launch turtlebot4_navigation localization.launch.py map:=./maps/mini_paa.yaml params:=./turtlebot4_bringup/config/localization.config.yaml`
4) Wait for the Turtlebot4 to initialize, using RViz2, set the initial
pose of the robot using the "Set Initial Pose" Tab at the top, using the map.
5) Once the Turtlebot4 and it's laser scan appear on screen, open the
navigation stack. This uses our own composed nav2 launch (single process)
instead of `turtlebot4_navigation nav2.launch.py` (11 processes), because the
discovery burst of 11 nodes registering with the discovery server on the
robot's Pi can starve AMCL's bond heartbeat and make the localization
lifecycle manager shut everything down
(`CRITICAL FAILURE: SERVER amcl IS DOWN`). The param file defaults to
`turtlebot4_bringup/config/nav2.config.yaml`:
`ros2 launch turtlebot4_bringup nav2.launch.py`
6) Once the Navigation stack is up and ready to roll with the costmap
showing on screen, we can run our Navigation node:
`ros2 run turtlebot4_custom_py nav_to_node`

### Troubleshooting
- **Robot disappears from RViz when starting nav2 / `CRITICAL FAILURE: SERVER amcl IS DOWN`**:
  the localization lifecycle manager stopped receiving AMCL's heartbeat (4 s timeout)
  because discovery/CPU got overloaded. Make sure you are using the composed launch in
  step 5, check laptop CPU with `htop`, and check the Wi-Fi link to the robot with
  `ping 192.168.50.223` while nav2 starts.
- **Nav2 hangs at "Waiting for service ... get_state"**: discovery traffic is not
  flowing. Verify the discovery server on the robot is reachable (`ping 192.168.50.223`)
  and that stale ROS processes from earlier runs are killed.
