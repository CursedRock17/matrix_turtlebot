# Navigate to a Goal
This is a simple set of instructions to navigate to a goal with a Matrix Turtlebot4

## Prerequisites
This tutorial assumes you've gone ahead and generate a map with SLAM, we now want to use 
localization

### Steps:
1) Open 4 Separate terminals, source each one of them for the project:
`source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash && source turtlebot4_bringup/setup.bash`
2) In the First Terminal, we want to run our visualization stack with RViz2:
`ros2 launch turtlebot4_viz view_navigation.launch.py namespace:=/matrix_turtlebot1`
3) In the Second Terminal, we want to run the localization layer:
`ros2 launch turtlebot4_navigation localization.launch.py namespace:=/matrix_turtlebot1 map:=./maps/lucas_room.yaml`
4) Wait for the Turtlebot4 to initialize, using RViz2, set the initial
pose of the robot using the "Set Initial Pose" Tab at the top, using the map.
5) Once the Turtlebot4 and it's laser scan appear on screen, open the 
navigation stack:
`ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/matrix_turtlebot1`
6) Once the Navigation stack is up and ready to roll with the costmap
showing on screen, we can run our Navigation node:
`ros2 run turtlebot4_custom_py nav_to_node`

