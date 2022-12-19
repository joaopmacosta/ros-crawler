#! /bin/bash

# Source ROS environment.
source devel/setup.bash

# Start ROS IMC broker.
cd src/crawler_teleop/launch & /opt/ros/melodic/bin/roslaunch crawler_teleop teleop.launch