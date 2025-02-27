#!/usr/bin/env python3
# high level functions for moving EB head
# computes offsets for neck angles, and limits to safe positions


import sys
import roslib
import rospy, time
#from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed
from eb_servos.servo_joint_list import *

from eb_servos.servo_joint_list import head_joints
from eb_servos.head_servo_publishers import *
from eb_servos.srv import ReturnJointStates


class HeadControl():
    def __init__(self, name):
        self.calling_name = name

        # CONSTANTS
        self.MAX_PAN = 1.047       #  60 degrees
        self.MAX_SIDETILT = 0.40   #  max before hitting neck
        self.MAX_TILT_UP =  0.70   #  0.65 # 0.52    #  Limit vertical to assure good tracking
        self.MAX_TILT_DOWN = 1.10  #  max down before hitting neck
        self.MAX_ANTENNA_BACK = -0.9      
        self.MAX_ANTENNA_FORWARD = 1.5

    # Track current servo positions
    def call_return_joint_states(self, joint_names):
        rospy.wait_for_service("return_joint_states")
        try:
            s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
            resp = s(joint_names)
        except rospy.ServiceException as e:
            rospy.logwarn("%s: Error when calling return_joint_states: %s" % (self.calling_name, e))
        for (ind, joint_name) in enumerate(joint_names):
            if(not resp.found[ind]):
                rospy.logwarn("%s: joint %s not found!" % (self.calling_name, joint_name ))
        return (resp.position, resp.velocity, resp.effort)


    def robot_is_sleeping(self):
        # Only enable activity if the robot is not in sleep position
        (current_sidetilt, current_pan, current_tilt, current_neck) = self.get_head_servo_positions()
        sleeping = False
        if current_neck <= -0.95:
            # robot is in sleep position
            sleeping = True
            rospy.loginfo('%s: WARNING ROBOT IS SLEEPING - movement disabled! Neck = %f' % (self.calling_name, current_neck))
        return sleeping


    def get_head_servo_positions(self):
        # returns positions of servos. Tilt normalized with neck position (so 0 = facing straight)
        (position, velocity, effort) = self.call_return_joint_states( \
            ['head_sidetilt_joint', 'head_pan_joint', 'head_tilt_joint', 'neck_raise_joint'])

        sidetilt = position[0]
        pan = position[1]
        tilt = position[2]
        neck = position[3]

        #print( " DBG:HeadControl: *** Current Raw Positions: sidetilt = %2.3f, pan = %2.3f, tilt = %2.3f, neck = %2.3f" % 
        #    (sidetilt, pan, tilt, neck))

        # factor in neck positon for tilt
        adjusted_tilt = tilt - neck
        #print( " DBG:HeadControl: *** get_head_servo_positions raw Tilt = %2.3f, Adjusted Tilt = %2.3f" % (tilt, adjusted_tilt))

        return(sidetilt, pan, adjusted_tilt, neck)

    def servos_are_moving(self):

        (position, velocity, effort) = self.call_return_joint_states( \
            ['head_sidetilt_joint', 'head_pan_joint', 'head_tilt_joint', 'neck_raise_joint'])
        # velocity threshold is .09 at slow speed, but go even less
        
        if( (velocity[0] + velocity[1]  + velocity[2] + velocity[3]) > 0.05 ):
            return True
        else:
            return False


    def antenna_move(self, command, antenna = "both"):
        
        #print( "DBG:HeadControl: move antenna called: command = %2.3f" %  (command))
        #return
        if command > self.MAX_ANTENNA_FORWARD:
            print( "DBG:HeadControl: limiting antenna to MAX. Requested = %2.3f" %  (command))
            command = self.MAX_ANTENNA_FORWARD
        elif command < self.MAX_ANTENNA_BACK:
            print( "DBG:HeadControl: limiting antenna to -MAX. Requested = %2.3f" %  (command))
            command = self.MAX_ANTENNA_BACK
        if not self.robot_is_sleeping():
            if antenna == "both" or antenna == "right":
                pub_right_antenna.publish(command)
            if antenna == "both" or antenna == "left":
                pub_left_antenna.publish(command)


    def head_pan_move(self, command):
        
        #print( "DBG:HeadControl: head_pan called: command = %2.3f" %  (command))
        #return

        #start_time = time.time()
        
        if command > self.MAX_PAN:
            # print( "DBG:HeadControl: limiting pan to MAX. Requested = %2.3f" %  (command))
            command = self.MAX_PAN
        elif command < -self.MAX_PAN:
            # print( "DBG:HeadControl: limiting pan to -MAX. Requested = %2.3f" %  (command))
            command = -self.MAX_PAN
        if not self.robot_is_sleeping():
            pub_head_pan.publish(command)

        #elapsed_time = time.time() - start_time # in seconds
        #print( "DBG:HeadControl: PAN Elapsed Time = %2.4f" %  (elapsed_time))
    
    def head_sidetilt_move(self, command):
        # print( "DBG:HeadControl: head_sidetilt called: command = %2.3f" %  (command))
        #return
        if command > self.MAX_SIDETILT:
            # print( "DBG:HeadControl: limiting sidetilt to MAX. Requested = %2.3f" %  (command))
            command = self.MAX_SIDETILT
        elif command < -self.MAX_SIDETILT:
            # print( "DBG:HeadControl: limiting sidetilt to -MAX. Requested = %2.3f" %  (command))
            command = -self.MAX_SIDETILT
        if not self.robot_is_sleeping():
            pub_head_sidetilt.publish(command)

    def head_tilt_move(self, command):
        #print( "DBG:HeadControl: head_tilt_move called: command = %2.3f" %  (command))
        #return
        
        #start_time = time.time()
        
        if command > self.MAX_TILT_UP:
            # print( "DBG:HeadControl: limiting tilt to MAX_TILT_UP. Requested = %2.3f" %  (command))
            command = self.MAX_TILT_UP
       
        # normalized to neck position        
        (current_sidetilt, current_pan, current_tilt, current_neck) = self.get_head_servo_positions()
        new_tilt = current_neck + command
        # ---------------------------  
        
        #print " DBG:HeadControl: head_tilt_move called: command = %2.3f, neck = %2.3f, new_tilt = %2.3f" % (command, current_neck, new_tilt)

        # apply limits
        if command < -self.MAX_TILT_DOWN:
            # print( "DBG:HeadControl: limiting tilt to MIN. Requested = %2.3f" %  (command))
            command = self.MAX_TILT_DOWN
        
        if new_tilt < -self.MAX_TILT_DOWN:
            # print( "DBG:HeadControl: limiting tilt to MIN. Requested = %2.3f" %  (new_tilt))
            new_tilt = -self.MAX_TILT_DOWN
       
        if not self.robot_is_sleeping():
            pub_head_tilt.publish(new_tilt)


        #elapsed_time = time.time() - start_time # in seconds
        #print( "DBG:HeadControl: TILT Elapsed Time = %2.4f" %  (elapsed_time))


        


        

