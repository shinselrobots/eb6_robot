# eb_pose_buttons

## Introduction
-  For Sheldon, a "pose button" in each arm is monitored by that arm's Arduino
-  the Arduino publishes a message that is monitored by this module

-  For EB, Joystick buttons allow servos to be positioned and recorded

## Function
-  Button Down:  Relax all servos in that servo group (right_arm, left_arm, etc.) to allow positioning
-  Button Up:    
   - Set 'torque on' for all servos in that group, locking them into position
   - Print out current position for each servo in the group to aid setting the pose in behaviors

