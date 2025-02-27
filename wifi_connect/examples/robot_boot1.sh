#!/bin/bash
echo -n "Launching robot Booter Node"
cd /home/system
# NOTE: .bashrc has stupid "don't run this non interactively" in first ten lines!
# So this fails: . /home/system/.bashrc
eval "$(cat ~/.bashrc | tail -n +10)"
#echo -n $PATH
#env
#echo -n "Here goes..."
roslaunch eb robot_boot1.launch
#echo -n "Done Launching ROS booter node"

