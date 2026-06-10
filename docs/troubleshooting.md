
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
