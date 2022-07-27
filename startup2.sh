#!/bin/sh

gnome-terminal -- bash -c "roslaunch full_amr_sensor full.launch"
sleep 7
gnome-terminal -- bash -c "~/Desktop/VirtualEnv/start_nodejs.sh"

ssh jhtran@10.0.0.3
