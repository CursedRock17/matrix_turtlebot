# Time Synchronization (NTP/Chrony)
The Turtlebot4 system has **three clocks** and all of them stamp ROS messages:

```
 Laptop (NTP master)          RPi 4 (client + relay)         Create 3 base
 192.168.50.185        <--->  192.168.50.223          <--->  192.168.186.2
 chrony, stratum 10    WiFi   chrony                  usb0   busybox ntpd
                              (192.168.186.3 on usb0)
```

- The **laptop** is the time master. BaleNet has no internet, so the laptop just
  serves its own clock (`local stratum 10`). Absolute time may be wrong, but
  everyone agreeing with each other is what matters for TF.
- The **RPi** syncs to the laptop over WiFi and re-serves time to the Create 3
  over the internal usb0 network (`192.168.186.0/24`).
- The **Create 3** runs a picky busybox ntpd pointed at the RPi
  (`192.168.186.3`). It stamps `/odom` and the `odom -> base_link` TF, so if
  its clock is off, navigation is dead even when RViz looks healthy.

## Config

### Laptop (`/etc/chrony/chrony.conf`)
```
local stratum 10
allow 192.168.50.0/24
```

### RPi (`/etc/chrony/chrony.conf`)
```
server 192.168.50.185 prefer iburst minpoll 4 maxpoll 6
allow 192.168.50.0/24
allow 192.168.186.0/24
```
**Do NOT add `local stratum 11` here.** The RPi has no RTC: right after boot,
before it reaches the laptop, its clock is garbage (roughly the image build
date). `local` makes chrony serve that garbage to the Create 3, which obeys.
Without `local`, chrony refuses to serve time until it has actually synced to
the laptop, and the Create 3 just keeps its own clock until then — much better.

After editing: `sudo systemctl restart chrony`

### Create 3 (web UI)
Open the Create 3 webserver (`http://192.168.50.223:8080` from the laptop, or
`http://192.168.186.2` from the RPi), go to Beta Features -> Edit ntp.conf, and
make sure it contains:
```
server 192.168.186.3
```

## Pre-flight check (do this before driving)
From a sourced terminal on the laptop:
```
ros2 topic delay /odom
```
Should settle around **0.01–0.05 s**. If it's large (seconds to millions of
seconds), the Create 3 clock hasn't synced yet — wait or power-cycle the robot,
don't bother launching nav.

On the RPi (`ssh ubuntu@192.168.50.223`):
```
chronyc sources      # want '^*' next to 192.168.50.185 and Reach = 377
chronyc tracking     # offsets should be in the millisecond range
```

## Case study: the 88-day clock jump (June 9, 2026)
Captured in `bags/fifth_bag/`. The Create 3 booted with roughly correct time,
asked the RPi for NTP before the RPi had synced to the laptop, and the RPi
(which still had `local stratum 11` at the time) served its unsynced boot
clock:
```
ntpd: setting time to 2026-03-13 16:52:11 (offset -7607277.7s)
```
For the next ~30 minutes every `/odom` message and odom TF was stamped 88 days
in the past (`ros2 topic delay /odom` read 7,610,410 s) and navigation was
impossible. At 18:30 UTC the Create 3 finally got good time and jumped forward
88 days. Removing `local stratum 11` from the RPi config prevents this.
