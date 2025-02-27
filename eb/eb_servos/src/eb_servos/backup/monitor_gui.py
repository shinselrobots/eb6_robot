#!/usr/bin/env python3

import rospy
import logging
#import time
#import math
#import sys
#import signal
#import numpy as np

#from sensor_msgs.msg import Joy
#from dynamixel_msgs.msg import JointState # single servo messages
#from dynamixel_msgs.msg import JointStateArray

from std_msgs.msg import Int32
#from std_msgs.msg import Bool
#from geometry_msgs.msg import Point32
from std_msgs.msg import Float32

import PySimpleGUI as psg


# ROS
rospy.init_node('SliderGui')

pub_val1  = rospy.Publisher('/value1',  Float32, queue_size=1)
pub_val2  = rospy.Publisher('/value2',  Float32, queue_size=1)
pub_val3  = rospy.Publisher('/value3',  Float32, queue_size=1)
pub_val4  = rospy.Publisher('/value4',  Float32, queue_size=1)
pub_val5  = rospy.Publisher('/value5',  Float32, queue_size=1)
pub_val6  = rospy.Publisher('/value6',  Float32, queue_size=1)
pub_val7  = rospy.Publisher('/value7',  Float32, queue_size=1)
pub_val8  = rospy.Publisher('/value8',  Float32, queue_size=1)
pub_val9  = rospy.Publisher('/value9',  Float32, queue_size=1)
pub_val10 = rospy.Publisher('/value10', Float32, queue_size=1)
pub_mode  = rospy.Publisher('/walk_mode', Int32, queue_size=1)

# GUI Layout
psg.set_options(font=('Arial Bold', 12))
layout = [
    [
        psg.Button("B0", enable_events=True, key='-B0-'),
        psg.Button("B1", enable_events=True, key='-B1-'),
        psg.Button("B2", enable_events=True, key='-B2-'),
        psg.Button("B3", enable_events=True, key='-B3-'),
    ],
    
    [
        psg.Text('Some text', size=(15, 1), expand_x=False, justification='left',
            # font=('Arial Bold', 12), relief="raised", border_width=5, expand_y= True,
        ), 
    ],

    [    
        psg.Text('1 Master', size=(15, 1), expand_x=False, justification='left',
            # font=('Arial Bold', 12), relief="raised", border_width=5, expand_y= True,
        ), 
        psg.Text('0', enable_events=True,
            key='-W0-', size=(3, 1), 
            expand_x=False, justification='center',
        ), 
        psg.Button("RST", enable_events=True, key='-R0-'),
    ],
    
]

# Main
window = psg.Window('Robot Walk Parameters', layout, size=(715, 600), element_justification='c')
val1 = 0
val2 = 0
val3 = 0
val4 = 0

while True:
    event, values = window.read()
    # print(event, values)
    
    if event == psg.WIN_CLOSED or event == 'Exit':
        break
        
    if event == '-R0-':
        val1 = val1 + 1
        window['-W0-'].update(val1)

        
        
window.close()

