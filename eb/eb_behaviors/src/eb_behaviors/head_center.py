#! /usr/bin/env python

import rospy
import time
import random
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Empty
from angles import fabs, shortest_angular_distance

# for servos
#from eb_servos.head_servo_publishers import *
#from eb_servos.right_arm_servo_publishers import *
#from eb_servos.left_arm_servo_publishers import *

from eb_servos.set_pose import *
#from eb_servos.standard_servo_positions import *
#from eb_servos.set_servo_speed import *
#from eb_servos.set_servo_torque import *

class HeadCenterBehavior():

    def __init__(self, behavior_utils, interrupt_check):
        self.module_name = 'head_center_behavior'
        rospy.loginfo('%s: Initializing...' % (self.module_name))
        self.send_status_update = behavior_utils.send_status_update
        self.speak = behavior_utils.speak

        self.pub_head_pan = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
        self.pub_head_tilt = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)
        self.pub_head_sidetilt = rospy.Publisher('/head_sidetilt_joint/command', Float64, queue_size=1)
        self.robot_is_sleeping = True
        self.TILT_SLIGHTLY = 0.15   #  tilt camera slightly up
        self.TILT_MEDIUM = 0.30         #  tilt camera more up
        self.TILT_HIGH = 0.60      #  tilt camera high for close encounters
        self.MAX_TILT_UP = 0.52    #  Limit vertical to assure good tracking
        self.MAX_TILT_DOWN = 1.10  #  max down before hitting neck

        self.tilt_up_state = 0              
        rospy.loginfo("%s: init complete." % (self.module_name))


    # Track current servo positions (need neck position to determine head tilt)
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


    def get_servo_positions(self):
        # returns positions of servos. Tilt normalized with neck position (so 0 = facing straight)
        (position, velocity, effort) = self.call_return_joint_states( \
            ['head_sidetilt_joint', 'head_pan_joint', 'head_tilt_joint', 'neck_raise_joint'])

        sidetilt = position[0]
        pan = position[1]
        tilt = position[2]
        neck = position[3]
        # factor in neck positon for tilt
        adjusted_tilt = tilt - neck
        return(sidetilt, pan, adjusted_tilt, neck)


    def head_pan_move(self, command):
        self.pub_head_pan.publish(command)
    
    def head_sidetilt_move(self, command):
        self.pub_head_sidetilt.publish(command)

    def head_tilt_move(self, command):
        if command > self.MAX_TILT_UP:
            command = self.MAX_TILT_UP
       
        # normalized to neck position        
        (current_sidetilt, current_pan, current_tilt, current_neck) = self.get_servo_positions()
        new_tilt = current_neck + command
        
        # apply limits
        if command < -self.MAX_TILT_DOWN:
            command = self.MAX_TILT_DOWN
        
        if new_tilt < -self.MAX_TILT_DOWN:
            new_tilt = -self.MAX_TILT_DOWN
       
        if not self.robot_is_sleeping:
            self.pub_head_tilt.publish(new_tilt)

    
    def cleanup(self):
        rospy.loginfo('%s: Behavior complete' % self.module_name)


    def execute_behavior(self, param1, param2):
        rospy.loginfo('%s: Executing behavior' % (self.module_name))
        rospy.loginfo( "Param1: '%s'", param1)
        rospy.loginfo( "Param2: '%s'", param2)
        

        # Only enable activity if the robot is not in sleep position
        (current_sidetilt, current_pan, current_tilt, current_neck) = self.get_servo_positions()
        if current_neck <= -0.95:
            # robot is in sleep position
            rospy.logwarn('%s: WARNING ROBOT IS SLEEPING - All movement disabled! Neck = %f' % (self.module_name, current_neck))
            self.cleanup
            return
        else:
            self.robot_is_sleeping = False

        if param1 == 'UP':
            self.tilt_up_state = self.tilt_up_state + 1
            if self.tilt_up_state > 2:
               self.tilt_up_state = 2

        elif param1 == 'DOWN':
            self.tilt_up_state = self.tilt_up_state - 1
            if self.tilt_up_state < 0:
               self.tilt_up_state = 0

        else:
            # cycle through tilt amounts
            self.tilt_up_state = self.tilt_up_state + 1
            if self.tilt_up_state > 2:
               self.tilt_up_state = 0 # start over
            
        tilt_amount = 0.0
        if self.tilt_up_state == 0:
            tilt_amount = self.TILT_SLIGHTLY # tilt head slightly up
            rospy.loginfo("%s: LOOKING SLIGHTLY UP" % (self.module_name))
                
        elif self.tilt_up_state == 1:
            tilt_amount = self.TILT_MEDIUM # tilt head up a bit more to find people more easily
            rospy.loginfo("%s: LOOKING MEDIUM UP" % (self.module_name))
                
        else:
            tilt_amount = self.TILT_HIGH # tilt head up high for close encounters
            rospy.loginfo("%s: LOOKING HIGH UP" % (self.module_name))


        # Center Camera Head
        SetServoSpeed(0.5, neck_joints)
        self.head_pan_move(0.0)
        self.head_tilt_move(tilt_amount)
        self.head_sidetilt_move(0.0)
        
        time.sleep(1.0) # seconds
        self.cleanup()         

        
if __name__ == '__main__':
    rospy.init_node('head_center_behavior')
    server = HeadCenterBehavior()
    rospy.spin()
