#! /usr/bin/env python

import rospy
import time
import random
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Empty

# for servos
#from eb_servos.head_servo_publishers import *
#from eb_servos.right_arm_servo_publishers import *
#from eb_servos.left_arm_servo_publishers import *

from eb_servos.set_pose import *
#from eb_servos.standard_servo_positions import *
#from eb_servos.set_servo_speed import *
#from eb_servos.set_servo_torque import *

class PoseBehavior():
    # Move robot into pose position that was requested

    def __init__(self, behavior_utils, interrupt_check):
        self.module_name = 'pose_behavior'
        rospy.loginfo('%s: Initializing...' % (self.module_name))

        self.interrupt_check = interrupt_check
        self.send_status_update = behavior_utils.send_status_update
        self.speak = behavior_utils.speak
        self.robotpose = RobotPose(self.module_name)

        rospy.loginfo("%s: init complete." % (self.module_name))


    def cleanup(self):
        rospy.loginfo('%s: Behavior complete' % self.module_name)

    # ----------------------------------------------------------------
    def execute_behavior(self, param1, param2):
        rospy.loginfo('%s: Executing behavior' % (self.module_name))
        rospy.loginfo( "Param1: '%s'", param1) # Pose
        rospy.loginfo( "Param2: '%s'", param2) # Servo Speed

        #self.speak("Ok")
        goal_pose = 0 
        lowest_servo_speed = 0.3

        # Param2 sets movement speed (optional) "x" means default
        if param2 != 'x' and param2 != '': 
            # Set custom speed
        
            rospy.loginfo("%s: Speed requested: %s" % (self.module_name, param2))
            speed_requested = float(param2)
            if speed_requested > 0.1:
                rospy.loginfo("%s: Setting Speed to %f" % (self.module_name, speed_requested))
                lowest_servo_speed = speed_requested

        # Param1 sets pose height: pose 0 - 5 (sleep, sit, mid-low, mid (ready), mid-high, high)
        rospy.loginfo("%s: Pose requested: %s" % (self.module_name, param1))
        
        
        if param1 == 'HIGH' or param1 == '5': # Stand Tall
            goal_pose = 5
            
        elif param1 == 'MID_HIGH' or param1 == '4': # Stand Medium High
            goal_pose = 4

        elif param1 == 'MID' or param1 == '3':
            goal_pose = 3

        elif param1 == 'MID_LOW' or param1 == '2': # Stand Medium Low
            goal_pose = 2
        
        elif param1 == 'SIT' or param1 == 'LOW' or param1 == '1': # Low
            goal_pose = 1
        
        elif param1 == 'SLEEP' or param1 == '0': # Low, with head sleep
            goal_pose = 0
            

        elif param1 == 'DOWN' or param1 == '10': # Move down one pose step
            self.robotpose.pose_down(lowest_servo_speed, False) # Don't center head when stepping up/down
            self.cleanup()         
            return
        
        elif param1 == 'UP' or param1 == '11': # Move up one pose step
            self.robotpose.pose_up(lowest_servo_speed, False) 
            self.cleanup()         
            return
        
        else: # ERROR
            rospy.logwarn('%s: ERROR - Unknow Pose Goal [%s] ignored!' % (self.module_name, param1))
            self.cleanup()         
            return
 

        # Move the servos
        self.robotpose.move(goal_pose, lowest_servo_speed)


        self.cleanup()         

        
if __name__ == '__main__':
    rospy.init_node('pose_behavior')
    server = PoseBehavior(None)
    rospy.spin()
