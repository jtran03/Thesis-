#!/bin/sh

gnome-terminal -- bash -c "roslaunch zlac8015d_motor_driver full.launch"
sleep 7
gnome-terminal -- bash -c "~/Desktop/VirtualEnv/start_nodejs.sh"
ssh jhtran@10.0.0.3 
1324
~/Desktop/startup.sh
