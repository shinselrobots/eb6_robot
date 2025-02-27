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

from std_msgs.msg import Int16
#from std_msgs.msg import Bool
#from geometry_msgs.msg import Point32
#from std_msgs.msg import Float64

import PySimpleGUI as psg


# ROS
rospy.init_node('SliderGui')
pub_val1 = rospy.Publisher('/value1', Int16, queue_size=1)
pub_val2 = rospy.Publisher('/value2', Int16, queue_size=1)
pub_val3 = rospy.Publisher('/value3', Int16, queue_size=1)
pub_val4 = rospy.Publisher('/value4', Int16, queue_size=1)
pub_val5 = rospy.Publisher('/value5', Int16, queue_size=1)
pub_val6 = rospy.Publisher('/value6', Int16, queue_size=1)
pub_val7 = rospy.Publisher('/value7', Int16, queue_size=1)
pub_val8 = rospy.Publisher('/value8', Int16, queue_size=1)

# GUI Layout
psg.set_options(font=('Arial Bold', 12))
layout = [
    # Slider 1 (shown with some options)
    [    
        psg.Text('Slider 1', size=(8, 1), expand_x=False, justification='left',
            # font=('Arial Bold', 12), relief="raised", border_width=5, expand_y= True,
        ), 
        psg.Slider(range=(-100, 100), default_value=0,
            expand_x=True, enable_events=True, 
            orientation='horizontal', key='-SL1-',
            # expand_y= True, tick_interval=5, disable_number_display=True, size=(15, 10),
        ),
        psg.Text('0', enable_events=True,
            key='-TXT1-', size=(3, 1), 
            expand_x=False, justification='center',
        ), 
        psg.Button("RST", enable_events=True, key='-Z1-'),
    ],
    
    # Slider 2
    [    
        psg.Text('Slider 2', size=(8, 1), expand_x=False, justification='left', ), 
        psg.Slider(key='-SL2-', range=(-100, 100), default_value=0, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        psg.Text('0', key='-TXT2-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        psg.Button("RST", enable_events=True, key='-Z2-'),
    ],
    
    # Slider 3
    [    
        psg.Text('Slider 3', size=(8, 1), expand_x=False, justification='left', ), 
        psg.Slider(key='-SL3-', range=(-100, 100), default_value=0, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        psg.Text('0', key='-TXT3-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        psg.Button("RST", enable_events=True, key='-Z3-'),
    ],
    
    # Slider 4
    [    
        psg.Text('Slider 4', size=(8, 1), expand_x=False, justification='left', ), 
        psg.Slider(key='-SL4-', range=(-100, 100), default_value=0, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        psg.Text('0', key='-TXT4-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        psg.Button("RST", enable_events=True, key='-Z4-'),
    ],
    
    # Slider 5
    [    
        psg.Text('Slider 5', size=(8, 1), expand_x=False, justification='left', ), 
        psg.Slider(key='-SL5-', range=(-100, 100), default_value=0, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        psg.Text('0', key='-TXT5-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        psg.Button("RST", enable_events=True, key='-Z5-'),
    ],
    
    # Slider 6
    [    
        psg.Text('Slider 6', size=(8, 1), expand_x=False, justification='left', ), 
        psg.Slider(key='-SL6-', range=(-100, 100), default_value=0, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        psg.Text('0', key='-TXT6-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        psg.Button("RST", enable_events=True, key='-Z6-'),
    ],
 
     # Slider 7
    [    
        psg.Text('Slider 7', size=(8, 1), expand_x=False, justification='left', ), 
        psg.Slider(key='-SL7-', range=(-100, 100), default_value=0, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        psg.Text('0', key='-TXT7-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        psg.Button("RST", enable_events=True, key='-Z7-'),
    ],
    
    # Slider 8
    [    
        psg.Text('Slider 8', size=(8, 1), expand_x=False, justification='left', ), 
        psg.Slider(key='-SL8-', range=(-100, 100), default_value=0, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        psg.Text('0', key='-TXT8-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        psg.Button("RST", enable_events=True, key='-Z8-'),
    ],
    

 
 
 
 
 
 
 
 
    
]

# Main
window = psg.Window('Parameters', layout, size=(715, 450))
val1 = 0
val2 = 0
val3 = 0
val4 = 0
val5 = 0
val6 = 0
val7 = 0
val8 = 0

while True:
    event, values = window.read()
    # print(event, values)
    
    if event == psg.WIN_CLOSED or event == 'Exit':
        break
        
    if event == '-SL1-':
        val1 = int(values['-SL1-'])
        window['-TXT1-'].update(val1)
        pub_val1.publish(Int16(val1))
    if event == '-Z1-':
        val1 = 0
        window['-SL1-'].update(val1)
        window['-TXT1-'].update(val1)
        pub_val1.publish(Int16(val1))

    if event == '-SL2-':
        val2 = int(values['-SL2-'])
        window['-TXT2-'].update(val2)
        pub_val2.publish(Int16(val2))
    if event == '-Z2-':
        val2 = 0
        window['-SL2-'].update(val2)
        window['-TXT2-'].update(val2)
        pub_val2.publish(Int16(val2))
        
    if event == '-SL3-':
        val3 = int(values['-SL3-'])
        window['-TXT3-'].update(val3)
        pub_val3.publish(Int16(val3))
    if event == '-Z3-':
        val3 = 0
        window['-SL3-'].update(val3)
        window['-TXT3-'].update(val3)
        pub_val3.publish(Int16(val3))
        
    if event == '-SL4-':
        val4 = int(values['-SL4-'])
        window['-TXT4-'].update(val4)
        pub_val4.publish(Int16(val4))
    if event == '-Z4-':
        val4 = 0
        window['-SL4-'].update(val4)
        window['-TXT4-'].update(val4)
        pub_val4.publish(Int16(val4))
        
    if event == '-SL5-':
        val5 = int(values['-SL5-'])
        window['-TXT5-'].update(val5)
        pub_val5.publish(Int16(val5))
    if event == '-Z5-':
        val5 = 0
        window['-SL5-'].update(val5)
        window['-TXT5-'].update(val5)
        pub_val5.publish(Int16(val5))
        
    if event == '-SL6-':
        val6 = int(values['-SL6-'])
        window['-TXT6-'].update(val6)
        pub_val6.publish(Int16(val6))
    if event == '-Z6-':
        val6 = 0
        window['-SL6-'].update(val6)
        window['-TXT6-'].update(val6)
        pub_val6.publish(Int16(val6))
        
    if event == '-SL7-':
        val7 = int(values['-SL7-'])
        window['-TXT7-'].update(val7)
        pub_val7.publish(Int16(val7))
    if event == '-Z7-':
        val7 = 0
        window['-SL7-'].update(val7)
        window['-TXT7-'].update(val7)
        pub_val7.publish(Int16(val7))
        
    if event == '-SL8-':
        val8 = int(values['-SL8-'])
        window['-TXT8-'].update(val8)
        pub_val8.publish(Int16(val8))
    if event == '-Z8-':
        val8 = 0
        window['-SL8-'].update(val8)
        window['-TXT8-'].update(val8)
        pub_val8.publish(Int16(val8))
        
        
        
window.close()

