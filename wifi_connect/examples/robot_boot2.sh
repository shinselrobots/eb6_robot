#!/bin/bash
#echo -n "Launching ROS"
#echo -n whoami
cd /home/system
# NOTE: .bashrc has stupid "don't run this non interactively" in first ten lines!
# So this fails: . /home/system/.bashrc
eval "$(cat ~/.bashrc | tail -n +10)"
#echo -n $PATH
#env
#echo "PID = " $$
#echo -n "Here goes..."
roslaunch eb robot_boot2.launch
#echo -n "Done Launching ROS"

