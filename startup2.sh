#!/bin/sh

gnome-terminal -- bash -c "roslaunch zlac8015d_motor_driver full.launch"
sleep 7
gnome-terminal -- bash -c "~/Desktop/VirtualEnv/start_nodejs.sh"

ssh jhtran@10.0.0.3 
spawn ssh jhtran@10.0.0.3 
expect "Password:*"
send "1324\r"
expect "$ "
interact
~/Desktop/startup.sh
