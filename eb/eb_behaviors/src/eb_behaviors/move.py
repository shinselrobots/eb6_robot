#! /usr/bin/env python

import rospy
import time
import math
import random
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Empty

# For wheel movement
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Point32, Pose, Quaternion, Twist, Vector3
from angles import * # Python dir() to see list

# for servos
#from eb_servos.head_servo_publishers import *
#from eb_servos.right_arm_servo_publishers import *
#from eb_servos.left_arm_servo_publishers import *

from eb_servos.set_pose import *
#from eb_servos.standard_servo_positions import *
#from eb_servos.set_servo_speed import *
#from eb_servos.set_servo_torque import *

from eb_behaviors.move_utils import MoveUtils


DEFAULT_MOVE_SPEED = 0.40 
MIN_MOVE_SPEED = 0.20

class MoveBehavior():
    # Move specified amount forward or back
    # intent: MOVE
    # param1: 0.5   # distance in meters (positive = forward)
    # param2: 0.5   # speed 0-1.0

    
    def __init__(self, behavior_utils, interrupt_check):
        self.module_name = 'move_behavior'
        rospy.loginfo('%s: Initializing...' % (self.module_name))
        self.interrupt_check = interrupt_check
        self.send_status_update = behavior_utils.send_status_update
        self.speak = behavior_utils.speak
        
        self.last_odom = 0.0
        self.goal_move_speed = 0.0
        self.goal_move_distance = 0.0
        self.slow_move_speed = MIN_MOVE_SPEED # slow down at the end for precision
        
        self.move_utils = MoveUtils(self.interrupt_check)
             
        rospy.loginfo("%s: init complete." % (self.module_name))


    def cleanup(self):
        rospy.loginfo('%s: Behavior complete' % self.module_name)


    # -------------------------------------------------------------------------
    def execute_behavior(self, param1, param2):
        rospy.loginfo('%s: Executing behavior' % (self.module_name))
        rospy.loginfo( "Param1: '%s'", param1) # # distance in meters (positive = forward)
        rospy.loginfo( "Param2: '%s'", param2) # move speed from MIN_MOVE_SPEED to 1.0 (always positive)

        move_amount = 0.0
        move_speed = DEFAULT_MOVE_SPEED

        try:
            move_amount = float(param1)
        except:
            rospy.logwarn('%s: Bad value [%s] for move Amount. Move cancelled.' % (self.module_name, param1))
            self.cleanup()
            return
                    
        if param2 == 'X' or param2 == '':
            move_speed = DEFAULT_MOVE_SPEED
        else:
            try:
                move_speed = fabs(float(param2))
            except:
                rospy.logwarn('%s: Bad value [%s] for Move   Speed. Using DEFAULT_MOVE_SPEED' % (self.module_name, param2))

        # Begin moving
        status_ok = self.move_utils.begin_move(move_amount, move_speed)
        if not status_ok: 
            return # problem beginning the move


        # Robot is moving! Monitor the move progress
        
        loopcount = 0
        while not rospy.is_shutdown():
            
            if self.interrupt_check():
                rospy.loginfo('%s: Interrupt Detected. Exiting Behavior.' % self.module_name)
                break
                
            if self.move_utils.check_move_complete():
                break

            # timeout in case odom not working right
            loopcount = loopcount + 1
            if loopcount > 4 * 10:  # number of seconds until timeout
                rospy.logwarn( "MOVE BEHAVIOR: TIMED OUT!  <<<<<<<<<<<<<<<<<<<<<<<<<<")                 
                break
            
            rospy.sleep(0.1)

        # Finish Behavior
        rospy.sleep(0.25)
        self.move_utils.check_move_complete() # Display how accurately we stopped
        self.cleanup()         

        
if __name__ == '__main__':
    rospy.init_node('move_behavior')
    server = MoveBehavior(None)
    rospy.spin()
