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
from eb_servos.head_control import HeadControl
#from eb_servos.head_servo_publishers import *
#from eb_servos.right_arm_servo_publishers import *
#from eb_servos.left_arm_servo_publishers import *

#from eb_servos.set_pose import *
#from eb_servos.standard_servo_positions import *
from eb_servos.set_servo_speed import *
from eb_servos.set_servo_torque import *

from eb_behaviors.move_utils import TurnUtils

DEFAULT_TURN_SPEED = 0.50 # half max
MIN_TURN_SPEED = 0.20
TURN_STOP_FUDGE = 0.080 # radians

class TurnBehavior():
    # Turn specified amount. NOTE: Positive = Left (per "right hand rule")
    
    def __init__(self, behavior_utils, interrupt_check):
        self.module_name = 'turn_behavior'
        rospy.loginfo('%s: Initializing...' % (self.module_name))
        self.interrupt_check = interrupt_check
        self.send_status_update = behavior_utils.send_status_update
        self.speak = behavior_utils.speak
        
        self.turn_utils = TurnUtils(self.interrupt_check)
        self.head_control = HeadControl(self.module_name)
        
        rospy.loginfo("%s: init complete." % (self.module_name))

            
    def publish_motor_command(self):
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = self.goal_turn_speed
        #print("DBG: sending TWIST command: ", twist)
        self.pub_wheel_motors.publish(twist)
    
    def stop_turn(self):
        # stop any wheel motion
        self.goal_turn_speed = 0.0
        self.publish_motor_command()
        print("TWIST STOP!")

    def cleanup(self):
        self.turn_utils.stop()
        rospy.loginfo('%s: Behavior complete' % self.module_name)


    # -------------------------------------------------------------------------
    def execute_behavior(self, param1, param2):
        rospy.loginfo('%s: Executing behavior' % (self.module_name))
        rospy.loginfo( "Param1: '%s'", param1) # +/- turn amount in degrees. Positive = left
        rospy.loginfo( "Param2: '%s'", param2) # Turn speed from MIN_TURN_SPEED to 1.0 (always positive)

        turn_amount = 0.0
        turn_speed = DEFAULT_TURN_SPEED

        try:
            turn_amount = float(param1)
        except:
            rospy.logwarn('%s: Bad value [%s] for Turn Amount. Turn cancelled.' % (self.module_name, param1))
            self.cleanup()
            return
                    
        if param2 == 'X' or param2 == '':
            turn_speed = DEFAULT_TURN_SPEED
        else:
            try:
                turn_speed = fabs(float(param2))
            except:
                rospy.logwarn('%s: Bad value [%s] for Turn Speed. Using DEFAULT_TURN_SPEED' % (self.module_name, param2))


        # Begin turning
        status_ok = self.turn_utils.begin_turn(turn_amount, turn_speed)
        if not status_ok: 
            return # problem beginning the turn

        print("TURNING HEAD")
        # while the body is turning, turn head opposite to keep pointing about where it was
        # Get the current servo pan and tilt position, already normalized to neck position        
        (current_sidetilt, current_pan, current_tilt, current_neck) = self.head_control.get_head_servo_positions()

        # Pan the head
        pan_angle = math.radians(-turn_amount) # opposite of turn, in radians
        self.head_control.head_pan_move(pan_angle)

        # Compensate for Pan and Tilt to level the sidetilt
        sidetilt_angle = pan_angle * current_tilt * -1
        #smooth_sidetilt_angle = (.5 * current_sidetilt) + (.5 * sidetilt_angle)            
        
        self.head_control.head_sidetilt_move(sidetilt_angle)     # Send servo command
        
        
        while not rospy.is_shutdown():
            
            if self.interrupt_check():
                rospy.loginfo('%s: Interrupt Detected. Exiting Behavior.' % self.module_name)
                break
                
            if self.turn_utils.check_turn_complete():
                break

            rospy.sleep(0.05)

        # Finish Behavior
        self.turn_utils.stop()
        rospy.sleep(0.25)
        self.turn_utils.check_turn_complete() # Display how accurately we stopped
        self.cleanup()         

        
if __name__ == '__main__':
    rospy.init_node('turn_behavior')
    server = TurnBehavior(None)
    rospy.spin()
