# Sim Running


### Setup
I had issues with the Lidar in GazeboSim since I'm not using a GPU and the lidar works better
with one. What you need to do is make the "Sensors" Engine "Ogre2" in either the robot
which is located in `/opt/ros/{ROS-DISTRO}/share/irobot_create_description`, find the
create3 `.xacro` file and alter from `ogre` to `ogre`.
