# Generate a Map

1) Install `turtlebot4_navigation`
    `sudo apt install ros-jazzy-turtlebot4-navigation`
2) Source local environment
    `source ./turtlebot4_bringup/setup.bash`
3) Launch synchronous SLAM with the `turtlebot4_navigation` package
    `ros2 launch turtlebot4_navigation slam.launch.py sync:=true params:=./turtlebot4_bringup/config/slam.config.yaml`
4) Launch RViz2 to view (make sure the terminal is sourced as well for domain ID)
    `ros2 launch turtlebot4_viz view_navigation.launch.py`
5) Drive the physical turtlebot with the remote
    `ros2 launch turtlebot4_bringup joy_teleop.launch.py`
6) Once map is filled out to fullest extent save (should be able to save in RViz)
   or
   `ros2 run nav2_map_server map_saver_cli -f "./maps/map_name" --ros-args -p map_subscribe_transient_local:=true`

Our robots run at the root namespace (see
[raspberry_pi_setup.md](./raspberry_pi_setup.md)), so no namespace arguments
are needed. If a robot is ever configured with a namespace, pass the same
value as `namespace:=<ns>` to every launch above and `-r __ns:=/<ns>` to the
`map_saver_cli` run.

## Survey the locations file while you're there
Navigation on the new map needs a `maps/<name>.locations.yaml` beside it
(format: [maps/robotics_lab.locations.yaml](../maps/robotics_lab.locations.yaml)),
and surveying it is much easier while you're still standing in the building.
The dock poses must be surveyed with the robot, but the named locations can
be clicked off the map: with the localization launch running on the new map,

    ros2 run turtlebot4_custom_py survey_locations

then type a name and click the spot in RViz with 'Publish Point' — it prints
a `locations:` block to paste into the file.

## Merging two maps into one
Big areas are easier to map in halves (Wi-Fi range, battery). The halves can
be merged offline afterwards:

    ros2 run turtlebot4_custom_py merge_maps \
        maps/first_floor.yaml maps/second_half_building.yaml \
        --dx <m> --dy <m> --dtheta <deg> -o maps/entire_building_merged

The two SLAM runs have unrelated frames, so you supply the transform: pick a
landmark visible in both maps (a doorway both runs drove through), read its
(x, y) off each map in RViz with 'Publish Point', start with the difference
as dx/dy, and iterate until the walls in the overlap line up instead of
doubling. Poses keep their meaning from the *first* (base) map, so its
locations file carries over; locations from the second map must be
re-surveyed on the merged map.
