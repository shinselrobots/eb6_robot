#!/usr/bin/env python3

import sys
import roslib
# roslib.load_manifest('eb_servos')
import rospy, time
import numpy as np
import math




from eb_servos.standard_servo_positions import *
from eb_servos.set_servo_speed import *
from eb_servos.set_servo_torque import *
from eb_servos.servo_joint_list import *
from eb_servos.srv import ReturnJointStates


class LegHeight():

    def __init__(self, name):
        self._calling_name = name


    def call_return_joint_states(self, joint_names):
        rospy.wait_for_service("return_joint_states")
        try:
            s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
            resp = s(joint_names)
        except rospy.ServiceException as e:
            rospy.logwarn("%s: Error when calling return_joint_states: %s" % (self._calling_name, e))
        for (ind, joint_name) in enumerate(joint_names):
            if resp.found == 0:
                rospy.logwarn("%s: joint %s not found!" % (self._calling_name, joint_name ))
        return (resp.position, resp.velocity, resp.effort)



    def move(self, target_height, lowest_servo_speed=0.3):

        ### WARNING! LEG HITS BODY WHEN ANGLE TO SMALL! TODO
        # I think 45 degrees is smallest angle?
        
        print( "DBG: set_leg_height: Starting ")
        # Set servos speed and torque
        #SetServoTorque(0.5, head_joints)
        #SetServoSpeed(0.35, head_joints) 
        # Calculate postions
        
        thigh_length = 144.38   # Constant. Actual bone lenght of thight and also shin
        # Total length hip to ankle is thigh_length + shin length, which happen to be the same
        
        # if target_height > thigh_length:
        #    print("ERROR: target heigth greater than leg length!")
        #    return
        

        opp_over_hyp = (target_height / 2.0) / thigh_length
        print("opp_over_hyp = ", opp_over_hyp)
        half_knee_angle = np.arcsin( opp_over_hyp)
        print("calculated half knee angle = ", half_knee_angle)
        degrees = np.degrees(half_knee_angle)
        print("degrees = ", degrees)
        
        knee_angle = half_knee_angle * 2.0
        print("calculated half knee angle = ", knee_angle)
        degrees = np.degrees(knee_angle)
        print("degrees = ", degrees)
        
        
        
        
        
        
        
        print( "LegHeight:Move: *********** DONE **********")

        
if __name__=='__main__':

    total = len(sys.argv)
    cmdargs = str(sys.argv)
    height = 0.0
    lowest_servo_speed = 0.3 # default

    set_leg_height = LegHeight("leg_height")
    rospy.init_node('leg_height', anonymous=True)
    if total > 1:
        height = float(sys.argv[1])
        if total > 2:
            lowest_servo_speed = float(sys.argv[2])

        set_leg_height.move(height, lowest_servo_speed)

    else:
        print( 'USAGE: set_leg_height.py <height in mm> <optional servo_speed (0.2 - 4.0)> ')
        #sys.exit()
        
        
        


