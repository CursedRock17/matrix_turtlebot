# Generate a Map

1) Install `turtlebot4_navigation`
    `sudo apt install ros-jazzy-turtlebot4-navigation`
2) Source local environment
    `source ./turtlebot4_bringup/setup.bash`
3) Launch synchronous SLAM with the `turtlebot4_navigation` package
    `ros2 launch turtlebot4_navigation slam.launch.py sync:=true params:=./turtlebot4_bringup/config/slam.config.yaml namespace:=matrix_turtlebot1`
4) Launch RViz2 to view (make sure the terminal is sourced as well for domain ID)
    `ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=matrix_turtlebot1`
5) Drive the physical turtlebot with the remote
    `ros2 launch turtlebot4_bringup joy_teleop.launch.py namespace:=matrix_turtlebot1`
6) Once map is filled out to fullest extent save (should be able to save in RViz)
   or
   `ros2 run nav2_map_server map_saver_cli -f "./maps/map_name" --ros-args -p map_subscribe_transient_local:=true -r __ns:=/matrix_turtlebot1`
