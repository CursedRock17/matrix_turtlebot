
### Troubleshooting
- **The nav2 launch never prints `Managed nodes are active` / stuck after `Activating bt_navigator`**:
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
- **`CRITICAL FAILURE: SERVER map_server IS DOWN` (or `amcl IS DOWN`) ~4 s after
  localization activates / robot disappears from RViz**: the localization lifecycle
  manager stopped receiving a server's bond heartbeat (4 s timeout) because the server's
  executor got blocked or CPU/discovery got overloaded. Most reproducible on **large
  maps** (`first_floor`, 2496×745 ≈ 1.86 MB) where map_server is busy sending the latched
  map. Fix: launch localization via **`ros2 launch turtlebot4_bringup localization.launch.py`**
  (our wrapper that disables the manager's bond watchdog with `SetParameter`), not
  `turtlebot4_navigation localization.launch.py` directly. Also make sure nav2 is the
  composed `nav2.launch.py` (not the 11-process one), and check laptop CPU with `htop`.
- **Nav2 hangs at "Waiting for service ... get_state"**: discovery traffic is not
  flowing. Verify the discovery server on the robot is reachable (`ping 192.168.50.223`)
  and that stale ROS processes from earlier runs are killed.
- **Robot drives but localization drifts or jumps**: run the time-sync pre-flight check
  (`ros2 topic delay /odom`, want ~0.01–0.05 s) — see [time_sync.md](./time_sync.md).
- **`Failed to activate global_costmap because transform from base_link to map did not
  become available` then `Failed to bring up all requested nodes. Aborting bringup`**:
  nothing is publishing the `map` frame yet — AMCL only broadcasts map→odom once it has
  laser scans (robot undocked) AND an initial pose (`nav_to_node` sets it). Start
  `nav_to_node` right after launching nav2 instead of waiting for activation;
  `initial_transform_timeout: 600.0` in nav2.config.yaml makes nav2 wait up to 10
  minutes for this instead of aborting at 60 s.

### Claude Notes
Hard-won one-liners from real debugging sessions, each tagged with the problem it solves.

- The TurtleBot4 switches the lidar off while docked, so always undock before calling `waitUntilNav2Active()`: **docked localization deadlock**
- The Create 3 undock backs off ~0.5 m and spins 180°, and the dock is only at `(0,0)` on maps whose SLAM run started there — the undock pose lives in each map's `maps/<name>.locations.yaml` and must be surveyed per map: **wrong initial pose**
- Nav2's global costmap can't activate until AMCL publishes the `map` frame, which on a docked robot only happens after the nav node undocks — fixed by `initial_transform_timeout: 600.0` under `global_costmap` in nav2.config.yaml (default 60 s aborted the whole bringup), plus starting the nav node concurrently instead of waiting: **costmap never created**
- Localization that dies ~4 s after `Managed nodes are active` is the bond watchdog, not the map or pose — a big map blocks map_server past the 4 s heartbeat; launch it via `turtlebot4_bringup localization.launch.py` (bond disabled), same trick as nav2: **localization self-destructs on big maps**
- AMCL only publishes `amcl_pose` after processing a laser scan, so an endless `Waiting for amcl_pose` means no `/scan` data is arriving: **localization stuck**
- Check the laptop's local discovery server with `ss -lun | grep 11888`, not pgrep — the processes are named `fastdds.py` and `fast-discovery-server`: **bt_navigator stall**
- Nav2 bonds are disabled with `SetParameter` in `nav2.launch.py` because a params-file section never reaches the composed lifecycle manager: **collision_monitor bond timeout**
- Full node logs live in `~/.ros/log/` even when a redirected terminal captured nothing, because `ros2 run` buffers stdout: **empty terminal capture**
- When a run hangs silently, check the previous run's container log in `~/.ros/log/` — an earlier attempt often logged the error the wedged one didn't: **silent hangs**
- `ros2 topic delay /odom` above ~0.05 s means the laptop→Pi→Create 3 chrony chain hasn't converged and TF lookups will fail with extrapolation errors: **time-sync drift**
- Kill stale ROS processes between test runs — orphan discovery servers hold port 11888 and leftover static TF publishers poison the real robot's TF tree: **ghost processes**
