# Issues Encountered
---------------------------
This document will list out a variety of issues I encountered along with the resolutions

| Issues | Solutions|
|--------|---------|
|Message Filter dropping message: frame 'rplidar_link' at time 1750840092.509 for reason 'the timestamp on the message is earlier than all the data in the transform cache | Setup NTP Server with Chrony |
|RPLidar stops on dock, mentioned on [Turtlebot4 GitHub](https://github.com/turtlebot/turtlebot4/blob/jazzy/turtlebot4_node/src/turtlebot4.cpp#L394) | No Solution |
|Battery Dies at 12%, mentioned on [Turtlebot4 GitHub](https://github.com/turtlebot/turtlebot4/blob/jazzy/turtlebot4_node/src/turtlebot4.cpp#L351) | No need for a solution, just be careful |


