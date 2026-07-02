# Raspberry Pi Setup (Turtlebot4)
How to correctly configure the RPi 4 on a MATRIX Lab Turtlebot4 once Ubuntu and
the turtlebot4 packages are flashed/installed. For the flashing/imaging steps
see [setup.md](./setup.md).

## 1) Network + ROS config with `turtlebot4-setup`
ssh into the RPi (`ssh ubuntu@192.168.50.224` on BaleNet, password `ubuntu`)
and run `turtlebot4-setup`. Configure:

- **Wi-Fi Setup**: connect to `BaleNet` (no internet on this network — that's
  fine, see [time_sync.md](./time_sync.md) for the consequences).
- **ROS Setup -> Discovery Server**: enabled, the RPi is the discovery server.
  Our laptops expect it at `192.168.50.224:11811` (see
  `turtlebot4_bringup/setup.bash`). **Verify it actually persists.** The server
  runs from `/usr/sbin/discovery` (`fastdds discovery -i 0 -p 11811`), but
  `turtlebot4-setup` has been seen to leave its systemd service *disabled* — so
  it survives only as a leftover process and vanishes on the next reboot (this
  cost us an entire evening after an IP change). After apply + reboot, confirm
  `ss -lunp | grep 11811` shows it listening; if not:
  `sudo systemctl enable --now discovery` (or install a unit that runs
  `fastdds discovery -i 0 -l 0.0.0.0 -p 11811`). The discovery script has no IP
  in it, so an IP change never breaks it directly — a reboot exposing the
  un-enabled service is what breaks it.
- **ROS Setup -> Bash Setup**:
  - `ROS_DOMAIN_ID`: `5`
  - `ROBOT_NAMESPACE`: leave **empty** for the single-robot setup — the docs
    assume the robot publishes `/scan`, `/odom`, etc. at the root namespace.
    The custom nodes and launches do support a namespace (for multi-robot):
    set it here, then pass the same value as `namespace:=<ns>` to every
    launch and as `--ros-args -r __ns:=/<ns>` to every `ros2 run` node.
- Apply settings; the RPi will restart its services.
- **Then verify both services are enabled, not just running**:
  `systemctl is-enabled turtlebot4 discovery` — both should say `enabled`.
  `turtlebot4.service` (the ROS bringup that starts the RPLIDAR etc.) has also
  been found disabled after setup (2026-07-01: the lidar wouldn't start until
  `sudo systemctl enable --now turtlebot4.service`). A service that's merely
  *running* dies on the next reboot.

The laptop side of this config lives in `turtlebot4_bringup/setup.bash`
(`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=5`,
`ROS_DISCOVERY_SERVER="192.168.50.224:11811;127.0.0.1:11888;"`,
`ROS_SUPER_CLIENT=True` in interactive shells) — source it in every terminal
that talks to the robot. Sourcing it also auto-starts a **second, local
discovery server** on the laptop (`fastdds discovery -i 1 -l 127.0.0.1 -p 11888`).
The local server lets laptop-side nodes (nav2, RViz, scripts) discover each
other without a Wi-Fi round trip to the Pi; without it, bt_navigator hangs
forever while activating (see
[navigate_to_a_goal.md](./navigate_to_a_goal.md)).

## 2) Time sync
Required, not optional — an unsynced clock silently breaks TF and navigation.
Follow [time_sync.md](./time_sync.md): chrony on the RPi syncs to the laptop
and serves the Create 3. In particular, make sure the RPi's chrony config does
**not** contain `local stratum 11`.

## 3) Verification checklist
After a fresh boot of the robot, from a sourced laptop terminal:

1. `ping 192.168.50.224` — RPi reachable on BaleNet.
2. `ros2 topic list` — should show the robot's topics (`/scan`, `/odom`,
   `/tf`, ...). If you only see `/parameter_events` and `/rosout`, the
   discovery server isn't reachable or your terminal isn't sourced.
3. `ros2 topic hz /scan` — RPLIDAR publishing at ~10 Hz.
4. `ros2 topic delay /odom` — **~0.01–0.05 s**. Anything bigger: the Create 3
   clock isn't synced yet (see [time_sync.md](./time_sync.md)).
5. Undock/dock from the laptop to prove the Create 3 listens:
   `ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"`

## Known failure modes
- **Empty `ros2 topic list`** (only `/rosout` + `/parameter_events`) — **even
  when shelled into the Pi**: the discovery server isn't running. See the
  persistence note in step 1: `ss -lunp | grep 11811`, then
  `sudo systemctl enable --now discovery`. If it's empty only on the *laptop*,
  that's the laptop env — `ping` the Pi and re-source
  `turtlebot4_bringup/setup.bash`.
- **Robot visible but nav unusable, message filters dropping everything**:
  clock skew — run the time sync pre-flight check.
- **Everything dies the moment nav2 starts** (`CRITICAL FAILURE: SERVER amcl
  IS DOWN`): discovery storm from launching many nodes at once over the
  discovery server. Use the composed launch
  (`ros2 launch turtlebot4_bringup nav2.launch.py`) — see
  [navigate_to_a_goal.md](./navigate_to_a_goal.md).
- **nav2 stalls forever at `Activating bt_navigator`** (never prints `Managed
  nodes are active`): the local discovery server on the laptop isn't running,
  so bt_navigator's behavior-tree clients wait on the Pi to match them over
  Wi-Fi. Check `ss -lun | grep 11888`, re-source
  `turtlebot4_bringup/setup.bash`, and relaunch nav2.
- **The lidar is off while the robot is docked** — that's a Turtlebot4
  feature, not a fault. No `/scan` means AMCL can't confirm a pose, so any
  script must undock *before* calling `waitUntilNav2Active()`
  (`nav_to_node` does this).
