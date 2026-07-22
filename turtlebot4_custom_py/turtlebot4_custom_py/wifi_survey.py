#!/usr/bin/env python3
"""Wi-Fi survey logger: maps link quality onto the map frame as the robot drives.

The robot doubles as a site-survey rover: every few seconds this samples the
wireless link (signal dBm, link quality, BSSID, tx bitrate) and pairs it with
the latest AMCL pose, appending CSV rows. Drive the patrol, then plot the CSV
over the map to see dead zones — and watch the `bssid` column: a change there
is an AP roam, the prime suspect for the corridor TF/scan stalls (the traffic
stalls for ~100 ms - 2 s while the radio re-associates).

Run this ON THE PI — its wlan0 is the link that carries scan/odom/TF, so the
laptop's radio tells us nothing. The custom package isn't installed there, but
this file is deliberately standalone (rclpy + geometry_msgs only):

    scp turtlebot4_custom_py/turtlebot4_custom_py/wifi_survey.py \
        ubuntu@192.168.50.224:~
    ssh ubuntu@192.168.50.224
    source /etc/turtlebot4/setup.bash
    python3 wifi_survey.py            # appends to ~/wifi_survey_YYYYMMDD.csv

Needs AMCL running (the laptop nav stack) to get poses; rows are still written
without a pose (empty x,y) so pure-connectivity timelines work too. Reads
/proc/net/wireless and `iw dev <iface> link` — no wireless-tools/iwconfig
dependency (Ubuntu server images often lack it).
"""
import os
import re
import subprocess
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped

SAMPLE_SEC = 3.0
CSV_HEADER = 'time,x,y,signal_dbm,link_quality,bitrate_mbit,bssid\n'

# amcl_pose is published latched (reliable + transient local); matching it
# hands us the most recent pose immediately on startup.
AMCL_QOS = QoSProfile(depth=1,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def read_proc_wireless(iface):
    """(link_quality, signal_dbm) for iface from /proc/net/wireless, or Nones."""
    try:
        with open('/proc/net/wireless') as f:
            for line in f:
                parts = line.split()
                if parts and parts[0] == iface + ':':
                    return float(parts[2].rstrip('.')), float(parts[3].rstrip('.'))
    except (OSError, ValueError, IndexError):
        pass
    return None, None


def _run_first_available(commands):
    """stdout of the first command whose binary exists, else ''."""
    for cmd in commands:
        try:
            return subprocess.run(cmd, capture_output=True, text=True,
                                  timeout=2).stdout
        except FileNotFoundError:
            continue
        except subprocess.TimeoutExpired:
            return ''
    return ''


def read_iw_link(iface):
    """(bssid, tx_bitrate_mbit) for iface, or Nones.

    Tries `iw`, `iwconfig`, then `wpa_cli` — no internet on BaleNet means we
    can't install a missing tool on the Pi, so take whichever is there
    (wpa_cli always is on a Wi-Fi machine; it just lacks the bitrate).
    """
    out = _run_first_available([
        ['iw', 'dev', iface, 'link'],
        ['iwconfig', iface],
        ['wpa_cli', '-i', iface, 'status'],
    ])
    bssid = re.search(
        r'(?:Connected to|Access Point:|bssid=)\s*([0-9A-Fa-f:]{17})', out)
    rate = re.search(r'(?:tx bitrate:|Bit Rate[=:])\s*([\d.]+)\s*Mb', out,
                     re.IGNORECASE)
    return (bssid.group(1).lower() if bssid else None,
            float(rate.group(1)) if rate else None)


class WifiSurvey(Node):

    def __init__(self):
        super().__init__('wifi_survey')
        self.declare_parameter('interface', 'wlan0')
        self.declare_parameter('sample_sec', SAMPLE_SEC)
        self.declare_parameter(
            'out', os.path.expanduser(
                '~/wifi_survey_' + datetime.now().strftime('%Y%m%d') + '.csv'))
        self.iface = self.get_parameter('interface').value
        self.out_path = self.get_parameter('out').value

        self.pose = None
        self.last_bssid = None
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._pose_cb, AMCL_QOS)

        new_file = not os.path.exists(self.out_path)
        self.out = open(self.out_path, 'a', buffering=1)  # line-buffered: survives a crash
        if new_file:
            self.out.write(CSV_HEADER)
        self.create_timer(float(self.get_parameter('sample_sec').value), self._sample)
        self.get_logger().info(f'Surveying {self.iface} -> {self.out_path}')

    def _pose_cb(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y)

    def _sample(self):
        quality, signal = read_proc_wireless(self.iface)
        bssid, bitrate = read_iw_link(self.iface)
        x, y = self.pose if self.pose else ('', '')
        fmt = lambda v: f'{v:.2f}' if isinstance(v, float) else (v or '')
        self.out.write(','.join([
            f'{time.time():.1f}', fmt(x), fmt(y), fmt(signal),
            fmt(quality), fmt(bitrate), bssid or '']) + '\n')

        if bssid != self.last_bssid and self.last_bssid is not None:
            self.get_logger().warn(
                f'AP ROAM: {self.last_bssid} -> {bssid or "DISCONNECTED"} '
                f'at ({fmt(x)}, {fmt(y)})')
        self.last_bssid = bssid


def main(args=None):
    rclpy.init(args=args)
    node = WifiSurvey()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.out.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
