#!/bin/sh

# ZED 2
gnome-terminal -- bash -c "roslaunch zed_wrapper zed2.launch"

sleep 7

# IMU 
gnome-terminal -- bash -c "roslaunch tinkerforge_imu_ros bricks_v3.launch"

# Web Page
gnome-terminal -- bash -c "roslaunch rosbridge_server rosbridge_websocket.launch"

# Motor Driver
gnome-terminal -- bash -c "rosrun zlac8015d_motor_driver zlac8015d_driver.py"

# RPLiDAR
echo "Starting RPLIDAR ######################################"
gnome-terminal -- bash -c "sshpass -p "1324" ssh -o StrictHostKeyChecking=no jhtran@10.0.0.3 'bash -is < ~/Desktop/startup.sh'"

# Node JS
gnome-terminal -- bash -c "~/Desktop/VirtualEnv/start_nodejs.sh"

# VOltage Sensors
gnome-terminal -- bash -c "rosrun rosserial_python serial_node.py /dev/ttyUSB1"

# RVIZ
rosrun rviz rviz -d ~/Desktop/test.rviz
#gnome-terminal -- "source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash;" 
