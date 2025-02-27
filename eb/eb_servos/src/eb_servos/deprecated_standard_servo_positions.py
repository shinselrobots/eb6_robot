#!/usr/bin/env python3
# 90 degrees = 1.57, 45 = 0.785

#import roslib
# roslib.load_manifest('eb_servos')
import rospy
#import time
from std_msgs.msg import Float64
from eb_servos.head_servo_publishers import *
from eb_servos.leg_servo_publishers import *

# TODO make global constants for standard positions!
# NECK_DOWN = -1.170
# NECK_UP = 0.0

# TODO - CALCULATE HEAD TILT BASED UPON NECK POSITON.
# IE, accept "0" for centered, regardless of neck position
# EG: Neck_pos = requested_pos +/- some calculation based upon current neck pos



def all_sit(): # all in sitting position
    print("-----> all_sit")
    #head_sleep()
    legs_sit()

def all_sleep(): # all in sleep position (sit and rest head)
    print("-----> all_sleep")
    legs_sit()
    head_sleep()


# HEAD

def head_center():
    print("-----> head_center")
    pub_head_sidetilt.publish(0.0)
    pub_head_tilt.publish(-0.3) # Tilt up a bit
    pub_head_pan.publish(0.0)
    # NOTE: does not change neck position!

def head_home():
    print("-----> head_home")
    pub_head_sidetilt.publish(0.0)
    pub_head_tilt.publish(-1.5708) # -1.5708 is Centered. TODO Tilt up a bit
    pub_head_pan.publish(0.0)
    pub_head_neck.publish(-1.170) # down position

def head_sleep(): #TODO change to head_rest
    print("-----> head_sleep")
    pub_head_sidetilt.publish(0.0)
    pub_head_tilt.publish(-1.224) # Resting on neck in down position.
    pub_head_pan.publish(0.0)
    pub_head_neck.publish(-1.187) # Resting on down bumper

def head_up_one():
    print("-----> head_up_one")
    pub_head_sidetilt.publish(0.0)
    pub_head_pan.publish(0.0)
    pub_head_neck.publish(-0.9) #Raised 1/2 inch
    pub_head_tilt.publish(-0.9) # Looking straight forward (depends upon neck angle!)
    # NOTE: does not change neck position!

def head_up_two():
    print("-----> head_up_one")
    pub_head_sidetilt.publish(0.0)
    pub_head_pan.publish(0.0)
    pub_head_neck.publish(-0.6) #Raised several inchs
    pub_head_tilt.publish(-0.6) # Looking straight forward (depends upon neck angle!)
    # NOTE: does not change neck position!



# SIT
def right_leg_sit():
    print("-----> right_leg_sit")
    pub_right_leg_hip_rotate.publish(0.0)
    pub_right_leg_thigh_lift.publish(-0.90)
    pub_right_leg_knee_bend.publish(2.52)

def left_leg_sit():
    print("-----> left_leg_sit")
    pub_left_leg_hip_rotate.publish(0.0)
    pub_left_leg_thigh_lift.publish(-0.90)
    pub_left_leg_knee_bend.publish(2.52)

def legs_sit():  # both legs TODO - THIS IS UNBALANCED MOTION!
    right_leg_sit()
    left_leg_sit()



# LEG POSE

def stand_ready(): 
    print("-----> stand_ready")

    pub_right_leg_hip_rotate.publish(0.0)
    pub_left_leg_hip_rotate.publish(0.0)

    pub_right_leg_thigh_lift.publish(0.0)
    pub_left_leg_thigh_lift.publish(0.0)

    pub_right_leg_knee_bend.publish(0.0)
    pub_left_leg_knee_bend.publish(0.0)




def stand_tall(): 
    print("-----> stand_ready")

    
    pub_right_leg_hip_rotate.publish(0.0)
    pub_left_leg_hip_rotate.publish(0.0)

    pub_right_leg_thigh_lift.publish(0.0)
    pub_left_leg_thigh_lift.publish(0.0)

    pub_right_leg_knee_bend.publish(0.0)
    pub_left_leg_knee_bend.publish(0.0)









