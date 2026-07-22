# OAK-D depth → costmap: seeing feet before touching them

## Why

The RPLIDAR scans one horizontal plane ~19 cm up. It sees shins (and the
robot avoids them — field-proven), but toes stick out below the plane and
are invisible until the bumper or IR sensors touch/almost-touch them. The
OAK-D Pro's stereo depth sees the **whole foot from meters away**. Feeding
its points into the costmaps makes feet first-class obstacles, and the
already-proven legs pipeline (mark → inflate → avoid → beep) does the rest.

Sensor coverage after this work:

| Range | Sensor | Layer |
|---|---|---|
| 0 cm (contact) | Bumper | `bump_to_cloud` mark |
| ~0–10 cm | IR proximity ×7 | `bump_to_cloud` mark (pre-contact) |
| 0.3–3 m, lidar plane | RPLIDAR | voxel/obstacle layer (legs, walls) |
| 0.3–2 m, full height | **OAK-D depth** | **voxel layer (feet — this doc)** |

## Design decision: where to convert depth to points

The trap to avoid: `depthimage_to_laserscan` on a level camera reads the
**floor** as an obstacle (rays below the horizon hit the ground at 1.5–2 m
and get projected as ranges). We need a real 3D cloud so the voxel layer can
drop ground points by height (`min_obstacle_height`), keeping only z ≥ 3 cm.

Two viable pipelines — pick after measuring live:

- **Phase 1 (laptop-side, no Pi changes):** Pi publishes low-res depth
  (compressed); laptop runs `depth_image_proc` → PointCloud2 → voxel layer.
  Cheap to try, easy to roll back. Risk: Wi-Fi bandwidth (~150–250 KB/s at
  320×200 @ 3 Hz compressed) on a link that already stalls.
- **Phase 2 (Pi-side, the endgame):** convert + crop + downsample on the Pi,
  publish a skinny cloud (only points with 0.03 < z < 0.5 m within 2 m).
  Kilobytes/s over Wi-Fi. Risk: Pi CPU (it already wedges its network stack;
  measure before and after with the endurance logger).

## Step 0 — live inventory (next time the robot is on)

```bash
ros2 topic list | grep oakd            # what does the Pi publish today?
ros2 topic info /oakd/stereo/image_raw # does stereo depth exist at all?
ros2 topic bw /oakd/rgb/preview/image_raw   # baseline camera bandwidth
ssh ubuntu@192.168.50.224 'apt list --installed 2>/dev/null | grep -E "depthimage|depth-image"'
```

If stereo isn't published, it must be enabled in the OAK-D launch on the Pi.
Do NOT edit `/etc/turtlebot4/` generated files — run a second camera launch
in its own terminal (the rplidar workaround pattern) or copy the launch into
`~/` and modify the copy.

If a Pi package is missing (no internet on BaleNet): `apt download
ros-jazzy-<pkg>` on the laptop, `scp` the .deb over, `sudo dpkg -i`.

## Phase 1 wiring (laptop)

```bash
# depth (rectified) -> PointCloud2 in the camera's optical frame
ros2 run depth_image_proc point_cloud_xyz_node --ros-args \
  -r image_rect:=/oakd/stereo/image_raw \
  -r camera_info:=/oakd/stereo/camera_info \
  -r points:=/oakd_points
```

Then enable the prepared `oakd_points` source in
`turtlebot4_bringup/config/nav2.config.yaml` (voxel layer — the block is
already there, commented) and rebuild. The voxel layer needs TF from the
cloud's frame to `odom`; the TB4 URDF publishes the `oakd_*` frames, verify
with `ros2 run tf2_ros tf2_echo odom oakd_rgb_camera_optical_frame`.

## Verification

1. Stand a shoe 1 m in front of the robot, below the lidar plane (nothing
   else near it). RViz: local costmap shows a lethal blob at the shoe with
   `/scan` unable to see it (toggle the laser display to confirm).
2. Send a goal behind the shoe: the path must curve around it.
3. `ros2 topic bw /oakd_points` and `htop` before/after — record both in
   `claude_logs/` so the CPU/bandwidth cost is a measured fact.
4. Watch for false floor obstacles (blobs appearing on flat ground ahead):
   raise `min_obstacle_height` to 0.05 if the camera extrinsics are a little
   off, or fix the mount pitch in the URDF.

## Rollback

Re-comment the `oakd_points` block, rebuild — the costmaps are back to
lidar-only. The conversion node is standalone; just stop it.
