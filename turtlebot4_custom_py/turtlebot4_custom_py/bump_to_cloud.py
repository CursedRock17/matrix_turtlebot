#!/usr/bin/env python3
"""Turn Create 3 bump hits into short-lived nav2 costmap obstacles.

Feet sit below the lidar plane, so the bumper is the ONLY sensor that ever
sees them — but a bump normally goes nowhere: the Create 3 firmware reflex
backs the robot up (that part is built in and always on), then nav2, whose
costmaps never heard about the bump, replans straight through the same spot.

This node closes that loop. Each BUMP hazard becomes a point ~0.20 m from the
robot center on the bumped side, latched in the odom frame and republished on
`bump_points` (PointCloud2) for BUMP_PERSIST_SEC. Both costmaps subscribe to
it as a marking-only source (see nav2.config.yaml), so for those seconds the
bumped spot is a real obstacle: MPPI keeps the keep-out radius away from it
and the planner routes around it instead of retrying the identical path.

The republish (not publish-once) matters: the lidar sees through the empty
space above a foot, so scan raytracing would clear a one-shot mark almost
immediately. Points expire on their own; a person who walks away stops
mattering ~10 s later.

It also marks PRE-contact: the Create 3's 7 front IR proximity sensors sit at
bumper height — the only sensor that sees feet BEFORE touching them. Raw
/ir_intensity readings over `ir_threshold` (parameter; field-calibrate against
real shoes) and firmware OBJECT_PROXIMITY hazards both become the same
short-lived obstacles, so the robot can reroute around a foot without contact.

It also BEEPS at people: whenever the collision monitor's VirtualBumper ring
reports something within its radius (a person's legs, usually), it chirps the
Create 3 speaker every couple of seconds — a human-audible "I see you, I'm
slowing" that also warns whoever's feet are nearest.

Every nav node (nav_to_node, nav_patrol_loop, patrol_with_llm) starts this
automatically in a background thread, so bump feedback is part of the base
stack, not an extra terminal. The standalone entry point remains for manual
RViz-goal sessions:

    ros2 run turtlebot4_custom_py bump_to_cloud

Verify the input chain (press a bumper): ros2 topic echo /hazard_detection
"""
import math

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from builtin_interfaces.msg import Duration
from irobot_create_msgs.msg import (AudioNote, AudioNoteVector, HazardDetection,
                                    HazardDetectionVector, IrIntensityVector)
from nav2_msgs.msg import CollisionMonitorState
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

BUMP_DISTANCE_M = 0.20    # obstacle placed this far from base_link (shell ~0.17)
BUMP_PERSIST_SEC = 10.0   # how long a bump stays an obstacle (long enough to replan)
PUBLISH_HZ = 5.0          # keep re-marking faster than scan raytrace clears
MERGE_RADIUS_M = 0.15     # a held bumper repeats at ~60 Hz; don't stack duplicates

PROXIMITY_POLYGON = 'VirtualBumper'  # collision monitor ring that means "someone close"
BEEP_PERIOD_SEC = 2.0                # chirp at most this often while they stay close
BEEP_FREQ_HZ = 880                   # A5 — audible, not alarming

# The 7 front IR proximity sensors sit at bumper height — the only sensor
# besides the bumper itself that can see FEET (below the lidar plane), and it
# sees them BEFORE contact. Readings are uncalibrated reflectance (int16,
# higher = closer); the threshold needs field calibration against real shoes:
# echo /ir_intensity with a foot ~5 cm from the bumper and set the parameter
# a bit below what you see (ros2 param set /bump_to_cloud ir_threshold <n>).
IR_THRESHOLD_DEFAULT = 600
IR_DISTANCE_M = 0.25      # IR sees a few cm past the shell; place the mark there

# Bearing of each Create 3 bumper zone / IR sensor, from the frame_id
# (radians, base_link frame, +x forward). Approximate is fine — the point
# just has to land on the correct side of the robot.
ZONE_BEARINGS = {
    'bump_left': math.radians(60.0),
    'bump_front_left': math.radians(30.0),
    'bump_front_center': 0.0,
    'bump_front_right': math.radians(-30.0),
    'bump_right': math.radians(-60.0),
    'ir_intensity_side_left': math.radians(65.0),
    'ir_intensity_left': math.radians(38.0),
    'ir_intensity_front_left': math.radians(20.0),
    'ir_intensity_front_center_left': math.radians(3.0),
    'ir_intensity_front_center_right': math.radians(-3.0),
    'ir_intensity_front_right': math.radians(-20.0),
    'ir_intensity_right': math.radians(-38.0),
}


class BumpToCloud(Node):

    def __init__(self):
        super().__init__('bump_to_cloud')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.points = []  # [(x_odom, y_odom, expiry_monotonic_stamp)]
        self.declare_parameter('ir_threshold', IR_THRESHOLD_DEFAULT)
        self.create_subscription(
            HazardDetectionVector, 'hazard_detection', self._hazards_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            IrIntensityVector, 'ir_intensity', self._ir_cb,
            qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, 'bump_points', 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)

        # Proximity beeper: the collision monitor already knows when someone
        # is inside the VirtualBumper ring — voice it through the Create 3.
        self.last_beep = 0.0
        self.audio_pub = self.create_publisher(AudioNoteVector, 'cmd_audio', 10)
        self.create_subscription(
            CollisionMonitorState, 'collision_monitor_state', self._monitor_cb, 10)
        self.get_logger().info(
            f'Bump hits -> obstacles on bump_points for {BUMP_PERSIST_SEC:.0f}s each; '
            f'beeping when {PROXIMITY_POLYGON} triggers')

    def _monitor_cb(self, msg: CollisionMonitorState):
        if msg.polygon_name != PROXIMITY_POLYGON:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_beep < BEEP_PERIOD_SEC:
            return
        self.last_beep = now
        beep = AudioNoteVector(append=False)
        beep.notes = [AudioNote(frequency=BEEP_FREQ_HZ,
                                max_runtime=Duration(nanosec=300_000_000))]
        self.audio_pub.publish(beep)

    def _robot_pose_odom(self):
        """(x, y, yaw) of base_link in odom, or None while TF is unavailable."""
        try:
            t = self.tf_buffer.lookup_transform('odom', 'base_link', Time())
        except Exception:  # tf2 raises several lookup/extrapolation types
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def _mark(self, frame_id, distance, label):
        """Latch one obstacle point `distance` ahead of the named sensor zone."""
        pose = self._robot_pose_odom()
        if pose is None:
            self.get_logger().warn(f'{label} detected but no odom->base_link TF yet',
                                   throttle_duration_sec=5.0)
            return
        rx, ry, yaw = pose
        bearing = ZONE_BEARINGS.get(frame_id, 0.0)
        px = rx + distance * math.cos(yaw + bearing)
        py = ry + distance * math.sin(yaw + bearing)
        if any(math.hypot(px - x, py - y) < MERGE_RADIUS_M
               for x, y, _ in self.points):
            return  # same held contact, already marked
        now = self.get_clock().now().nanoseconds * 1e-9
        self.points.append((px, py, now + BUMP_PERSIST_SEC))
        text = (f'{label} ({frame_id}) -> obstacle at odom ({px:.2f}, {py:.2f}) '
                f'for {BUMP_PERSIST_SEC:.0f}s')
        if label == 'BUMP':
            self.get_logger().warn(text)
        else:
            # IR fires continuously against the dock while charging — don't
            # let routine proximity marks flood the log the way a bump should.
            self.get_logger().info(text, throttle_duration_sec=5.0)

    def _hazards_cb(self, msg: HazardDetectionVector):
        # OBJECT_PROXIMITY comes from the same IR hardware as _ir_cb but
        # pre-thresholded by the firmware — take both paths, _mark dedupes.
        for d in msg.detections:
            if d.type == HazardDetection.BUMP:
                self._mark(d.header.frame_id, BUMP_DISTANCE_M, 'BUMP')
            elif d.type == HazardDetection.OBJECT_PROXIMITY:
                self._mark(d.header.frame_id, IR_DISTANCE_M, 'PROXIMITY')

    def _ir_cb(self, msg: IrIntensityVector):
        threshold = self.get_parameter('ir_threshold').value
        for reading in msg.readings:
            if reading.value >= threshold:
                self._mark(reading.header.frame_id, IR_DISTANCE_M, 'IR')

    def _publish(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.points = [p for p in self.points if p[2] > now]
        header = Header(frame_id='odom')
        header.stamp = self.get_clock().now().to_msg()
        # z=0.1 puts the point inside the costmaps' obstacle height band.
        cloud = point_cloud2.create_cloud(
            header,
            [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
             for i, n in enumerate('xyz')],
            [(x, y, 0.1) for x, y, _ in self.points])
        # Publish even when empty: keeps the costmap source fresh, and an
        # empty cloud is how expired bumps disappear.
        self.pub.publish(cloud)

    def thread_function(self):
        """Spin in a daemon thread — how the nav nodes embed this (monitors.py
        pattern), so bump feedback runs with ANY of them automatically."""
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()


def main(args=None):
    rclpy.init(args=args)
    node = BumpToCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
