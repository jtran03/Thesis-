#!/bin/sh

gnome-terminal -- bash -c "roslaunch zlac8015d_motor_driver full.launch 
gnome-terminal -- bash -c "sshpass -p "1324" ssh -o StrictHostKeyChecking=no jhtran@10.0.0.3 'bash -is < ~/Desktop/startup.sh'"
gnome-terminal -- bash -c "~/Desktop/VirtualEnv/start_nodejs.sh"
rosrun rviz rviz -d ~/Desktop/test.rviz
