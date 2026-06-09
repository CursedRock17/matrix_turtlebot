source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
[ -t 0 ] && export ROS_SUPER_CLIENT=True || export ROS_SUPER_CLIENT=False
export ROS_DOMAIN_ID=5
export ROS_DISCOVERY_SERVER="192.168.50.223:11811;"
#export ROBOT_NAMESPACE=matrix_turtlebot1
