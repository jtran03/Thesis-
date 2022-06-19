#!/bin/sh

gnome-terminal -- bash -c "roslaunch zed_wrapper zed2.launch"

sleep 5

gnome-terminal -- bash -c "roslaunch tinkerforge_imu_ros bricks_v3.launch"

gnome-terminal -- bash -c "roslaunch rosbridge_server rosbridge_websocket.launch"

gnome-terminal -- bash -c "rosrun zlac8015d talker.py"


#gnome-terminal -- "source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash;" 
