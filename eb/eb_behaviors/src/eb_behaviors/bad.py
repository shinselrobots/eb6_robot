#! /usr/bin/env python

import rospy
import random
from random import randint

import math
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Empty
from std_msgs.msg import Bool


import pygame


# for servos
#from eb_servos.head_servo_publishers import *
#from eb_servos.right_arm_servo_publishers import *
#from eb_servos.left_arm_servo_publishers import *

from eb_servos.set_pose import *
#from eb_servos.standard_servo_positions import *
#from eb_servos.set_servo_speed import *
#from eb_servos.set_servo_torque import *

#from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Point32, Pose, Quaternion, Twist, Vector3

#from eb_behaviors.move_utils import MoveUtils



# ========================================================================        
class BadBehavior():

    def __init__(self, behavior_utils, interrupt_check):
        self.module_name = 'bad_behavior'
        rospy.loginfo('%s: Initializing...' % (self.module_name))

        self.interrupt_check = interrupt_check
        self.send_status_update = behavior_utils.send_status_update
        self.speak = behavior_utils.speak
        self.robotpose = RobotPose(self.module_name)

        # constants
        self.MAX_PAN = 1.047       #  60 degrees
        self.DEGREES_90 = 1.50     #  Radians
        self.MAX_SIDETILT = 0.40   # max before hitting neck
        self.MAX_TILT_UP =  0.70   # 0.65 # 0.52    #  Limit vertical to assure good tracking
        self.MAX_TILT_DOWN = 1.10  #  max down before hitting neck
        self.INTRO_TILT_CENTER = 0.25 # RELATIVE TO NECK! 0.1 = Look mostly forward, .25 for theatre demos
        self.CONVERSATION_TILT_CENTER = 0.25 # 0.15    #  lean toward up  (since robot is on the ground)
        self.MAX_ANTENNA_BACK = -0.7807      
        self.MAX_ANTENNA_FORWARD = 1.5
        self.INTRO_ANTENNA_CENTER = -0.3


        # Publishers
        self.pub_wheel_motors = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_neck_raise = rospy.Publisher('/neck_raise_joint/command', Float64, queue_size=1)
        self.pub_head_pan = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
        self.pub_head_tilt = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)
        self.pub_head_sidetilt = rospy.Publisher('/head_sidetilt_joint/command', Float64, queue_size=1)
        self.pub_right_antenna = rospy.Publisher('/right_antenna_joint/command', Float64, queue_size=1)
        self.pub_left_antenna = rospy.Publisher('/left_antenna_joint/command', Float64, queue_size=1)

        
        rospy.loginfo("%s: init complete." % (self.module_name))

      

    def publish_wheel_move_command(self, speed):
        print("DBG:publish_wheel_move_command: Speed = ", speed)
        twist = Twist()
        twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        #print("DBG: sending TWIST command: ", twist)
        self.pub_wheel_motors.publish(twist)

    def publish_wheel_turn_command(self, speed):
        print("DBG:publish_wheel_turn_command: Speed = ", speed)
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = speed
        #print("DBG: sending TWIST command: ", twist)
        self.pub_wheel_motors.publish(twist)


    # TODO - Copied from IDLE.PY  Need to make a common library and share this code
    # Track current servo positions
    def call_return_joint_states(self, joint_names):
        rospy.wait_for_service("return_joint_states")
        try:
            s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
            resp = s(joint_names)
        except rospy.ServiceException as e:
            rospy.logwarn("%s: Error when calling return_joint_states: %s" % (self.module_name, e))
        for (ind, joint_name) in enumerate(joint_names):
            if(not resp.found[ind]):
                rospy.logwarn("%s: joint %s not found!" % (self.module_name, joint_name ))
        return (resp.position, resp.velocity, resp.effort)

    def get_head_servo_positions(self):
        # returns positions of servos. Tilt normalized with neck position (so 0 = facing straight)
        (position, velocity, effort) = self.call_return_joint_states( \
            ['head_sidetilt_joint', 'head_pan_joint', 'head_tilt_joint', 'neck_raise_joint'])

        sidetilt = position[0]
        pan = position[1]
        tilt = position[2]
        neck = position[3]

        #print( " DBG:Idle Behavior: *** Current Raw Positions: sidetilt = %2.3f, pan = %2.3f, tilt = %2.3f, neck = %2.3f" % 
        #    (sidetilt, pan, tilt, neck))

        # factor in neck positon for tilt
        adjusted_tilt = tilt - neck
        #print( " DBG:Idle Behavior: *** get_head_servo_positions raw Tilt = %2.3f, Adjusted Tilt = %2.3f" % (tilt, adjusted_tilt))

        return(sidetilt, pan, adjusted_tilt, neck)


    def antenna_move(self, command, antenna = "both"):
        
        # print( "DBG:Idle Behavior: move antenna called: command = %2.3f" %  (command))
        #return
        if command > self.MAX_ANTENNA_FORWARD:
            # print( "DBG:Idle Behavior: limiting pan to MAX. Requested = %2.3f" %  (command))
            command = self.MAX_ANTENNA_FORWARD
        elif command < self.MAX_ANTENNA_BACK:
            # print( "DBG:Idle Behavior: limiting pan to -MAX. Requested = %2.3f" %  (command))
            command = self.MAX_ANTENNA_BACK

        if antenna == "both" or antenna == "right":
            self.pub_right_antenna.publish(command)
        if antenna == "both" or antenna == "left":
            self.pub_left_antenna.publish(command)


    def head_pan_move(self, command):
        
        #print( "DBG:Idle Behavior: head_pan called: command = %2.3f" %  (command))
        #return
        if command > self.MAX_PAN:
            # print( "DBG:Idle Behavior: limiting pan to MAX. Requested = %2.3f" %  (command))
            command = self.MAX_PAN
        elif command < -self.MAX_PAN:
            # print( "DBG:Idle Behavior: limiting pan to -MAX. Requested = %2.3f" %  (command))
            command = -self.MAX_PAN
        self.pub_head_pan.publish(command)
    
    def head_sidetilt_move(self, command):
        # print( "DBG:Idle Behavior: head_sidetilt called: command = %2.3f" %  (command))
        #return
        if command > self.MAX_SIDETILT:
            # print( "DBG:Idle Behavior: limiting sidetilt to MAX. Requested = %2.3f" %  (command))
            command = self.MAX_SIDETILT
        elif command < -self.MAX_SIDETILT:
            # print( "DBG:Idle Behavior: limiting sidetilt to -MAX. Requested = %2.3f" %  (command))
            command = -self.MAX_SIDETILT
        self.pub_head_sidetilt.publish(command)

    def head_tilt_move(self, command):
        #print( "DBG:Idle Behavior: head_tilt_move called: command = %2.3f" %  (command))
        #return
        if command > self.MAX_TILT_UP:
            # print( "DBG:Idle Behavior: limiting tilt to MAX_TILT_UP. Requested = %2.3f" %  (command))
            command = self.MAX_TILT_UP
       
        # normalized to neck position        
        (current_sidetilt, current_pan, current_tilt, current_neck) = self.get_head_servo_positions()
        new_tilt = current_neck + command
        # ---------------------------  
        
        #print " DBG:Idle Behavior: head_tilt_move called: command = %2.3f, neck = %2.3f, new_tilt = %2.3f" % (command, current_neck, new_tilt)

        # apply limits
        if command < -self.MAX_TILT_DOWN:
            # print( "DBG:Idle Behavior: limiting tilt to MIN. Requested = %2.3f" %  (command))
            command = self.MAX_TILT_DOWN
        
        if new_tilt < -self.MAX_TILT_DOWN:
            # print( "DBG:Idle Behavior: limiting tilt to MIN. Requested = %2.3f" %  (new_tilt))
            new_tilt = -self.MAX_TILT_DOWN
       
        self.pub_head_tilt.publish(new_tilt)


    def sleep_check(self, sleep_time):
        if self.interrupt_check():
            print("")
            print("**************************************************")
            print("****** BAD ROBOT: interrupt detected! Ending Behavior!")
            print("**************************************************")
            print("")
            self.cleanup()
            return True
            
        # randomly move the antennas
        leftAntennaAmt = random.uniform(-0.5, 0.5)
        self.antenna_move(self.INTRO_ANTENNA_CENTER+leftAntennaAmt, "left")

        rightAntennaAmt = random.uniform(-0.5, 0.5)
        self.antenna_move(self.INTRO_ANTENNA_CENTER+rightAntennaAmt, "right")

        # Tilt head a little every so often                  
        side_tilt_amt = random.uniform(-0.3, 0.3)
        self.head_sidetilt_move(side_tilt_amt)
            
        rospy.sleep(sleep_time)


    def cleanup(self):
        rospy.loginfo('%s: Behavior complete' % self.module_name)


    # --------------------------------------------------------------------------------------------
    def execute_behavior(self, param1, param2):
        rospy.loginfo('%s: Executing behavior' % (self.module_name))
        rospy.loginfo( "Param1: '%s'", param1)
        rospy.loginfo( "Param2: '%s'", param2)


        # Center Camera Head
        self.head_pan_move(0.0)
        self.head_tilt_move(self.INTRO_TILT_CENTER) 
        self.head_sidetilt_move(0.0)
        self.antenna_move(self.INTRO_ANTENNA_CENTER, "both")
        #panAmt = random.uniform(-0.5, 0.5)
        #self.head_pan_move(panAmt)

        # Shake head
        self.head_tilt_move(self.INTRO_TILT_CENTER - 0.5)
        shake_amount = 0.4
        shake_delay = 0.4
        SetServoSpeed(1.0, neck_joints)  # 0.35,0 .5
        self.speak("I am sorry", False) # don't wait)
        for i in range(0,2):
            self.head_pan_move(shake_amount) 
            rospy.sleep(shake_delay)
            self.head_pan_move(-shake_amount) 
            rospy.sleep(shake_delay)

        self.head_pan_move(0.0) 
        self.head_tilt_move(0.0 )
        SetServoSpeed(0.5, neck_joints)  # 0.35,0 .5
        self.robotpose.move(1, 0.3) # Pose, Speed - down

        if self.sleep_check(2.0):
            return
        
       
        self.cleanup()         

        
if __name__ == '__main__':
    rospy.init_node('bad_behavior')
    server = BadBehavior()
    rospy.spin()
