#!/bin/sh

gnome-terminal -- "source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; roslaunch zed_wrapper zed2.launch" 
