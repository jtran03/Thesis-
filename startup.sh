#!/bin/sh
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
gnome-terminal -- "roslaunch zed_wrapper zed2.launch" 
