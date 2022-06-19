#!/bin/sh

# ZED 2
gnome-terminal -- bash -c "roslaunch zed_wrapper zed2.launch"

sleep 7

# IMU 
gnome-terminal -- bash -c "roslaunch tinkerforge_imu_ros bricks_v3.launch"

# Web Page
gnome-terminal -- bash -c "roslaunch rosbridge_server rosbridge_websocket.launch"

# Motor Driver
gnome-terminal -- bash -c "rosrun zlac8015d talker.py"

# RPLiDAR
echo "Starting RPLIDAR ######################################"
gnome-terminal -- bash -c "sshpass -p "1324" ssh -o StrictHostKeyChecking=no jhtran@10.42.0.212 'bash -is < ~/Desktop/startup.sh'"

# Node JS
gnome-terminal -- bash -c "~/Desktop/VirtualEnv/start_nodejs.sh"

# RVIZ
rosrun rviz rviz ~/Desktop/test.rviz
#gnome-terminal -- "source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash;" 
