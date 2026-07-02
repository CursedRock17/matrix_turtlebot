# Troubleshooting

Fast path: if the robot seems unresponsive at all, run the doctor first —
`ros2 run turtlebot4_custom_py preflight`. It checks the laptop↔robot link, the
discovery server, dock state, and `/odom` `/scan` `/battery_state` rates, and
drops a diagnostic bag in `claude_logs/`. Most of the tables below start there.

## Connectivity & discovery (robot ↔ laptop)

| Symptom | Cause | Fix |
|---|---|---|
| `ros2 topic list` shows only `/rosout` + `/parameter_events`, even shelled into the Pi | The Fast DDS discovery server isn't running on the Pi. `turtlebot4-setup` writes the service file but doesn't enable it, so it's a leftover process that dies on reboot | Make it persistent: enable `discovery.service` (or install one running `fastdds discovery -i 0 -l 0.0.0.0 -p 11811`), `sudo systemctl enable --now discovery`. Verify `ss -lunp \| grep 11811`. |
| Laptop sees no robot topics; nodes hang; nav2 spams `base_link to odom did not become available` | Robot is publishing nothing to the laptop — powered off/booting, `ROS_DISCOVERY_SERVER` unset, or discovery server down | `preflight`; power on and wait ~90 s; `ping 192.168.50.224`; re-source `turtlebot4_bringup/setup.bash`. It's a connection issue, not the code. |
| A nav node dead-ends at "Loaded location file" and hangs | Same as above — robot silent on the wire (`/odom`, `/dock_status` absent) | `preflight`; the `_wait_for_robot` guard now fails fast with an actionable message. |
| nav2 stuck at `Activating bt_navigator` / RViz: `navigate_to_pose action server is not available` | bt_navigator's BT action clients must be matched by a discovery server; over Wi-Fi to the Pi that stalls forever | Local discovery server on the laptop (`127.0.0.1:11888`), auto-started by `setup.bash`. Check `ss -lun \| grep 11888`; re-source if missing. |
| Everything broke after the robot's IP changed | The Pi is now `192.168.50.224` (was `.223`). The discovery script has no IP, but `ROS_DISCOVERY_SERVER` and docs do | Update the laptop `ROS_DISCOVERY_SERVER` in `setup.bash`. The Create 3 reaches the Pi over usb0 (`192.168.186.3`, unchanged), so its own config needs nothing. |

## Startup: localization & nav2

| Symptom | Cause | Fix |
|---|---|---|
| `CRITICAL FAILURE: SERVER map_server IS DOWN` (or `amcl IS DOWN`) ~4 s after `Managed nodes are active`; robot vanishes from RViz | The localization lifecycle manager's 4 s bond watchdog — a big map (first_floor, 2496×745 ≈ 1.86 MB) blocks map_server's executor past the heartbeat | Launch localization via `ros2 launch turtlebot4_bringup localization.launch.py` (our bond-disabled wrapper), NOT `turtlebot4_navigation`'s. |
| `Failed to activate global_costmap … transform base_link to map` → `Aborting bringup`; "have to run nav2 twice" | The global costmap needs the `map` frame, which AMCL only publishes after the nav node undocks and sets the initial pose | Start the nav node right after launching nav2 (don't wait for activation). `initial_transform_timeout: 600` in `nav2.config.yaml` lets it wait instead of aborting at 60 s. |
| nav2 hangs at "Waiting for service … get_state" | Discovery traffic not flowing | Verify the discovery server is reachable (`ping 192.168.50.224`) and kill stale ROS processes. |

## Navigation behaviour

| Symptom | Cause | Fix |
|---|---|---|
| Goals abort `Failed to make progress` / `getTransform … extrapolation into the future`; lidar spinning but `/scan` looks "stale" in RViz | Intermittent 1–2 s stalls in scan/tf/odom **delivery**, NOT clock skew (mean `/scan` age was 17 ms; it had 1.9 s gaps). From laptop CPU saturation and/or Wi-Fi | Reduce laptop load (`use_rviz:=false`, watch `htop`); long-term run nav2 on the Pi. The patrol loop holds instead of driving blind. |
| Robot drives but localization drifts or jumps | Time sync (laptop→Pi→Create 3 chrony chain) not converged | `ros2 topic delay /odom` (want 0.01–0.05 s); see [time_sync.md](./time_sync.md). |
| `/scan` publishes fine (7 Hz) but nav is dead for ~30 min: `collision_monitor … timestamps differ … Ignoring the source`, TF `extrapolation into the past` | A robot-side clock diverged mid-run (seconds to minutes); chrony corrects by *slewing*, so nav2 rejects every scan until it converges | `makestep 1 -1` in the Pi's chrony.conf (steps instantly). The patrol loop now holds on stamp skew and says so. Cause hunt: `journalctl -u chrony` on the Pi — see [time_sync.md](./time_sync.md) case study. |
| Patrol ends with `Failed to dock for charging` — robot stops short of the dock | The Create 3's IR docking only works from ~1 m with the dock in view; nav2 can deliver the approach pose with enough error to miss that window | The patrol loop retries docking (3×), re-navigating to the approach pose between tries. Survey `approach_pose` ~0.5–1 m squarely in front of the dock. |
| A single goal overruns and gets cancelled | Building-scale legs are long and slow | `GOAL_TIMEOUT_SEC` is 300 s; keep patrol legs short (the first_floor patrol is out-and-back so no leg is the full ~50 m). |
| Robot clips people's feet | Feet sit below the lidar plane — legs are detected, feet are not | Keep-out widened (`inflation_radius: 0.6`); note the Create 3 bumpers don't trigger on thin legs. |
| `Waiting for amcl_pose to be received` loops forever | AMCL has no laser scan — robot docked (lidar off) or silent | `ros2 topic hz /scan`; the nav nodes now spin the lidar up front, so rebuild + re-source if you still see this. |

## Lidar & sensors

| Symptom | Cause | Fix |
|---|---|---|
| `/scan` stops publishing while docked | The Create 3 powers the RPLIDAR off whenever it's docked — by design | Undock first (the nodes do), or `/start_motor`. This is NOT a hardware fault; docked bags showing `/scan` gaps with `is_docked=true` are expected. |
| Startup deadlocks — "waiting for costmap / lidar / initial pose", all circular | costmap ← scan ← lidar chicken-and-egg | `startup.py` calls `/start_motor` **first** (idempotent; never stops the lidar). The undock is only for the pose, not to wake the lidar. |
| Node fails ~10 s after undock with "no scan" | Lidar didn't restart on undock (known redock issue) | The node retries `/start_motor` automatically; power-cycle the robot if `/scan` stays silent. |

## Endurance & platform (Raspberry Pi)

| Symptom | Cause | Fix |
|---|---|---|
| After ~40 min, `/odom` + `/scan` stop, `/diagnostics` still looks healthy, `ping` hangs (not "unreachable") but the router shows the robot connected | Pi network stack wedged — onboard Wi-Fi (`brcmfmac`) under sustained streaming, thermal throttling, or USB-C power sag (the Create 3 powers the Pi over that cable) | Cool the Pi (heatsink/fan); log `vcgencmd measure_temp` / `get_throttled` / `free -m` over a run; inspect the USB-C cable. Long-term: less over-Wi-Fi traffic (nav2 on the Pi). |
| Discovery server / all topics vanish after a reboot | The discovery server was never enabled to auto-start | Persistent `discovery.service` — see the Connectivity table. |
| RPLIDAR (and the Pi's whole ROS stack) never comes up after boot | `turtlebot4.service` — the Pi's ROS bringup — found *disabled* (2026-07-01), same never-enabled disease as the discovery service | `ssh` in, `sudo systemctl enable --now turtlebot4.service`. After any `turtlebot4-setup` run, verify: `systemctl is-enabled turtlebot4 discovery`. |

## Debugging tips

- Full node logs live in `~/.ros/log/` even when a redirected terminal captured nothing — `ros2 run` buffers stdout. Read the file, not the terminal.
- When a run hangs silently, check the **previous** run's container log in `~/.ros/log/` — an earlier attempt often logged the error the wedged one didn't.
- Kill stale ROS processes between runs — orphan discovery servers hold port 11888 and leftover static TF publishers poison the real robot's TF tree.
- The laptop's local discovery server processes are named `fastdds.py` / `fast-discovery-server`, so check the port (`ss -lun | grep 11888`), not `pgrep`.
- `preflight` records a bag to `claude_logs/preflight_HHMMSS` every run — a failed startup always leaves the evidence.
