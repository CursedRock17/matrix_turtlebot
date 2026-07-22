source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
[ -t 0 ] && export ROS_SUPER_CLIENT=True || export ROS_SUPER_CLIENT=False
export ROS_DOMAIN_ID=5
# Server 0 is the discovery server on the robot's RPi. Server 1 is a local
# server on the laptop so laptop-side nodes (nav2, RViz, scripts) can match
# each other without a round trip over WiFi -- without it, bt_navigator hangs
# forever in 'Activating' waiting for its own BT action clients to be matched
# by the Pi (see docs/navigate_to_a_goal.md troubleshooting).
export ROS_DISCOVERY_SERVER="192.168.50.224:11811;127.0.0.1:11888;"
if ! ss -lun 2>/dev/null | grep -q ':11888 '; then
    (nohup fastdds discovery -i 1 -l 127.0.0.1 -p 11888 > /dev/null 2>&1 &)
fi
#export ROBOT_NAMESPACE=matrix_turtlebot1
