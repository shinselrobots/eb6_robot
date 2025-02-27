#!/usr/bin/env python3
# To RUN:
# export ROSCONSOLE_STDOUT_LINE_BUFFERED=1


import rospy
import logging
#import time
#import math
#import sys
#import signal
#import numpy as np


from std_msgs.msg import Int32
#from std_msgs.msg import Bool
#from geometry_msgs.msg import Point32
from std_msgs.msg import Float32

import PySimpleGUI as sg


# ROS
rospy.init_node('battery_log')
rospy.loginfo("--------------------------------------------------")
rospy.loginfo("Starting Battery log...")
rospy.loginfo("Make sure you: export ROSCONSOLE_STDOUT_LINE_BUFFERED=1")
rospy.loginfo("")

def battery_callback(data):
    battery_voltage = data.data
    #battery_voltage_str="%2.2f" % battery_voltage
    rospy.loginfo( "Battery Voltage = %2.2f" % battery_voltage)


battery_sub = rospy.Subscriber("/battery_voltage", Float32, battery_callback)

rospy.spin()


