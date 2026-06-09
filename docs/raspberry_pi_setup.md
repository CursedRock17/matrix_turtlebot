# Raspberry Pi Setup (Turtlebot4)
How to correctly configure the RPi 4 on a MATRIX Lab Turtlebot4 once Ubuntu and
the turtlebot4 packages are flashed/installed. For the flashing/imaging steps
see [setup.md](./setup.md).

## 1) Network + ROS config with `turtlebot4-setup`
ssh into the RPi (`ssh ubuntu@192.168.50.223` on BaleNet, password `ubuntu`)
and run `turtlebot4-setup`. Configure:

- **Wi-Fi Setup**: connect to `BaleNet` (no internet on this network — that's
  fine, see [time_sync.md](./time_sync.md) for the consequences).
- **ROS Setup -> Discovery Server**: enabled, the RPi is the discovery server.
  Our laptops expect it at `192.168.50.223:11811` (see
  `turtlebot4_bringup/setup.bash`).
- **ROS Setup -> Bash Setup**:
  - `ROS_DOMAIN_ID`: `5`
  - `ROBOT_NAMESPACE`: leave **empty**. All our docs, configs, and the custom
    nodes assume the robot publishes `/scan`, `/odom`, etc. at the root
    namespace.
- Apply settings; the RPi will restart its services.

The laptop side of this config lives in `turtlebot4_bringup/setup.bash`
(`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=5`,
`ROS_DISCOVERY_SERVER="192.168.50.223:11811;"`, `ROS_SUPER_CLIENT=True`) —
source it in every terminal that talks to the robot.

## 2) Time sync
Required, not optional — an unsynced clock silently breaks TF and navigation.
Follow [time_sync.md](./time_sync.md): chrony on the RPi syncs to the laptop
and serves the Create 3. In particular, make sure the RPi's chrony config does
**not** contain `local stratum 11`.

## 3) Verification checklist
After a fresh boot of the robot, from a sourced laptop terminal:

1. `ping 192.168.50.223` — RPi reachable on BaleNet.
2. `ros2 topic list` — should show the robot's topics (`/scan`, `/odom`,
   `/tf`, ...). If you only see `/parameter_events` and `/rosout`, the
   discovery server isn't reachable or your terminal isn't sourced.
3. `ros2 topic hz /scan` — RPLIDAR publishing at ~10 Hz.
4. `ros2 topic delay /odom` — **~0.01–0.05 s**. Anything bigger: the Create 3
   clock isn't synced yet (see [time_sync.md](./time_sync.md)).
5. Undock/dock from the laptop to prove the Create 3 listens:
   `ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"`

## Known failure modes
- **Empty `ros2 topic list`**: discovery server down or laptop env not
  sourced. Check `ping`, then re-source `turtlebot4_bringup/setup.bash`.
- **Robot visible but nav unusable, message filters dropping everything**:
  clock skew — run the time sync pre-flight check.
- **Everything dies the moment nav2 starts** (`CRITICAL FAILURE: SERVER amcl
  IS DOWN`): discovery storm from launching many nodes at once over the
  discovery server. Use the composed launch
  (`ros2 launch turtlebot4_bringup nav2.launch.py`) — see
  [navigate_to_a_goal.md](./navigate_to_a_goal.md).
