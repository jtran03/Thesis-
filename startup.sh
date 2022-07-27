#!/bin/sh

gnome-terminal -- bash -c "roslaunch simulink_amr start.launch"
sleep 7
gnome-terminal -- bash -c "~/Desktop/VirtualEnv/start_nodejs.sh"

ssh jhtran@10.0.0.3
#roslaunch zlac8015d_motor_driver zlac8015d_driver.py
