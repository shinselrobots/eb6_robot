# eb_behaviors
Behavior server and behaviors.
To test:
- shell 1: roslaunch eb behavior_test.launch
- shell 2: cd ~/catkin_robot/src/eb/eb_behaviors/src/eb_behaviors
  ./behavior_server.py
- shell 3: rostopic pub -1 /behavior/cmd behavior_common/CommandState -- "WAKE" "x" "x"

## NOTES:







  


