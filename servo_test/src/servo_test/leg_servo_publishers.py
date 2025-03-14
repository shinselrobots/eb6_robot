#!/usr/bin/env python3

import roslib
# roslib.load_manifest('eb_servos')
import rospy
from std_msgs.msg import Float64

# Servo Position Command Publishers

pub_right_leg_thigh_lift = rospy.Publisher('/right_leg_thigh_lift_joint/command', Float64, queue_size=1)
pub_right_leg_knee_bend = rospy.Publisher('/right_leg_knee_bend_joint/command', Float64, queue_size=1)
pub_right_leg_ankle_rotate = rospy.Publisher('/right_leg_ankle_rotate_joint/command', Float64, queue_size=1)

pub_left_leg_thigh_lift = rospy.Publisher('/left_leg_thigh_lift_joint/command', Float64, queue_size=1)
pub_left_leg_knee_bend = rospy.Publisher('/left_leg_knee_bend_joint/command', Float64, queue_size=1)
pub_left_leg_ankle_rotate = rospy.Publisher('/left_leg_ankle_rotate_joint/command', Float64, queue_size=1)


