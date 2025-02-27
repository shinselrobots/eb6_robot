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

import PySimpleGUI as sg


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
pub_val11 = rospy.Publisher('/value11', Float32, queue_size=1)
pub_val12 = rospy.Publisher('/value12', Float32, queue_size=1)
pub_mode  = rospy.Publisher('/walk_mode', Int32, queue_size=1)

# Global values
val1 = 0
val2 = 0
val3 = 0
val4 = 0
val5 = 0
val6 = 200 # default servo speed value
val7 = 0
val8 = 0
val9 = 0
val10 = 0
val11 = 0
val12 = 0



# GUI Layout
# sg.theme('DarkBlue13')
# button_background_color = ("", "")
# background_color=sg.theme_background_color()
sg.set_options(font=('Arial Bold', 12))
layout = [
    [
        sg.Text(' 0.00', enable_events=True, key='-BATTERY-', size=(5, 1), text_color = "light gray",
            expand_x=False, justification='left',  ), 
        sg.Text('Volts', size=(7, 1), expand_x=False, pad=(0,0)),

        sg.Button("P1", enable_events=True, key='-POSE0-', pad=(1,0)),
        sg.Button("P2", enable_events=True, key='-POSE4-', pad=(1,0)),
        sg.Button("P3", enable_events=True, key='-POSE4-', pad=(1,0)),
        sg.Button("P4", enable_events=True, key='-POSE4-', pad=(1,0)),
        sg.Button("STOP", enable_events=True,  key='-STOP-',  pad=(1,0)),
        sg.Button("WALK",  enable_events=True, key='-WALK-',  pad=(1,0)),
        sg.Text(' ', size=(3, 1), expand_x=False, ),
        sg.Button("RST",  enable_events=True, key='-RST-ALL-'),
    ],
    
    # Slider 1 (shown with some options)
    [    
        sg.Text('Master Dly', size=(12, 1), expand_x=False, justification='left',
            # font=('Arial Bold', 12), relief="raised", border_width=5, expand_y= True,
        ), 
        sg.Slider(key='-SL1-', range=(-100, 100), default_value=val1,
            enable_events=True, orientation='horizontal', expand_x=True,              
            # expand_y= True, tick_interval=5, disable_number_display=True, size=(12, 10),
        ),
        sg.Text('01', enable_events=True,
            key='-TXT1-', size=(3, 1), 
            expand_x=False, justification='center',
        ), 
        sg.Button("RST", enable_events=True, key='-Z1-'),
    ],
    
    # Slider 12
    [    
        sg.Text('Step Height', size=(12, 1), expand_x=False, justification='left', ), 
        sg.Slider(key='-SL12-', range=(-100, 100), default_value=val12, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        sg.Text('12', key='-TXT12-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        sg.Button("RST", enable_events=True, key='-Z12-'),
    ],

    # Slider 2
    [    
        sg.Text('Leg Trigger', size=(12, 1), expand_x=False, justification='left', ), 
        sg.Slider(key='-SL2-', range=(-100, 100), default_value=val2, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        sg.Text('02', key='-TXT2-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        sg.Button("RST", enable_events=True, key='-Z2-'),
    ],
    
    # Slider 6
    [    
        sg.Text('Servo Speed', size=(12, 1), expand_x=False, justification='left', ), 
        sg.Slider(key='-SL6-', range=(20, 420), default_value=val6, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        sg.Text('06', key='-TXT6-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        sg.Button("RST", enable_events=True, key='-Z6-'),
    ],
 
    # Slider 3
    [    
        sg.Text('MD StepTimer', size=(12, 1), expand_x=False, justification='left', ), 
        sg.Slider(key='-SL3-', range=(-100, 100), default_value=val3, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        sg.Text('03', key='-TXT3-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        sg.Button("RST", enable_events=True, key='-Z3-'),
    ],
    
    # Slider 4
    [    
        sg.Text('ID IMU_Trgr', size=(12, 1), expand_x=False, justification='left', ), 
        sg.Slider(key='-SL4-', range=(-100, 100), default_value=val4, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        sg.Text('04', key='-TXT4-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        sg.Button("RST", enable_events=True, key='-Z4-'),
    ],
    
    # Slider 5
    [    
        sg.Text('IMU Mul', size=(12, 1), expand_x=False, justification='left', ), 
        sg.Slider(key='-SL5-', range=(-100, 100), default_value=val5, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        sg.Text('05', key='-TXT5-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        sg.Button("RST", enable_events=True, key='-Z5-'),
    ],
    
    # Slider 7
    [    
        sg.Text('IMU Tilt', size=(12, 1), expand_x=False, justification='left', ), 
        sg.Slider(key='-SL7-', range=(-100, 100), default_value=val7, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        sg.Text('07', key='-TXT7-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        sg.Button("RST", enable_events=True, key='-Z7-'),
    ],

    # Slider 11
    [    
        sg.Text('IMU Pitch', size=(12, 1), expand_x=False, justification='left', ), 
        sg.Slider(key='-SL11-', range=(-200, 200), default_value=val11, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        sg.Text('11', key='-TXT11-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        sg.Button("RST", enable_events=True, key='-Z11-'),
    ],

    
    # Slider 8
    [    
        sg.Text('PID P', size=(12, 1), expand_x=False, justification='left', ), 
        sg.Slider(key='-SL8-', range=(0, 500), default_value=val8, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        sg.Text('08', key='-TXT8-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        sg.Button("RST", enable_events=True, key='-Z8-'),
    ],
    
    # Slider 9
    [    
        sg.Text('PID I', size=(12, 1), expand_x=False, justification='left', ), 
        sg.Slider(key='-SL9-', range=(0, 500), default_value=val9, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        sg.Text('09', key='-TXT9-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        sg.Button("RST", enable_events=True, key='-Z9-'),
    ],
    
    # Slider 10
    [    
        sg.Text('PID D', size=(12, 1), expand_x=False, justification='left', ), 
        sg.Slider(key='-SL10-', range=(-500, 500), default_value=val10, 
            enable_events=True, orientation='horizontal', expand_x=True, ),
        sg.Text('10', key='-TXT10-', enable_events=True,  
            size=(3, 1), expand_x=False, justification='center', ), 
        sg.Button("RST", enable_events=True, key='-Z10-'),
    ],

    
]


# Main
window = sg.Window('Robot Walk Parameters', layout, size=(600, 650), element_justification='l', finalize=True)

def battery_callback(data):
    battery_voltage = data.data
    battery_voltage_str="%2.2f" % battery_voltage
    battery_text_color = "green"
    if battery_voltage < 6.00:   # No battery 
       battery_text_color = "light gray" 
    elif battery_voltage < 12.58:   # 2 min 
       battery_text_color = "orange" 
    elif battery_voltage < 13.26:   # 5 min 
       battery_text_color = "orange"
    elif battery_voltage < 13.81:    # 10 min
       battery_text_color = "orange"
    elif battery_voltage < 14.36:    # 20 min
       battery_text_color = "red"
    elif battery_voltage < 14.60:    # 30 min
       battery_text_color = "yellow"
    elif battery_voltage < 14.88:    # 60 min
       battery_text_color = "white"

    window['-BATTERY-'].update(battery_voltage_str, text_color=battery_text_color)


#event, values = window.read() # must do before callback fires
battery_sub = rospy.Subscriber("/battery_voltage", Float32, battery_callback)


while True:
    event, values = window.read()
    # print(event, values)
    reset_all = False    
    if event == sg.WIN_CLOSED or event == 'Exit':
        break
 
    if event == '-RST-ALL-':
        reset_all = True
        
    if event == '-SL1-':
        val1 = int(values['-SL1-'])
        # window['-TXT1-'].update(val1)
        pub_val1.publish(Float32(val1))
    if event == '-Z1-' or reset_all:
        val1 = 0
        window['-SL1-'].update(val1)
        # window['-TXT1-'].update(val1)
        pub_val1.publish(Float32(val1))

    if event == '-SL2-':
        val2 = int(values['-SL2-'])
        # window['-TXT2-'].update(val2)
        pub_val2.publish(Float32(val2))
    if event == '-Z2-' or reset_all:
        val2 = 0
        window['-SL2-'].update(val2)
        # window['-TXT2-'].update(val2)
        pub_val2.publish(Float32(val2))
        
    if event == '-SL3-':
        val3 = int(values['-SL3-'])
        # window['-TXT3-'].update(val3)
        pub_val3.publish(Float32(val3))
    if event == '-Z3-' or reset_all:
        val3 = 0
        window['-SL3-'].update(val3)
        # window['-TXT3-'].update(val3)
        pub_val3.publish(Float32(val3))
        
    if event == '-SL4-':
        val4 = int(values['-SL4-'])
        # window['-TXT4-'].update(val4)
        pub_val4.publish(Float32(val4))
    if event == '-Z4-' or reset_all:
        val4 = 0
        window['-SL4-'].update(val4)
        # window['-TXT4-'].update(val4)
        pub_val4.publish(Float32(val4))
        
    if event == '-SL5-':
        val5 = int(values['-SL5-'])
        # window['-TXT5-'].update(val5)
        pub_val5.publish(Float32(val5))
    if event == '-Z5-' or reset_all:
        val5 = 0
        window['-SL5-'].update(val5)
        # window['-TXT5-'].update(val5)
        pub_val5.publish(Float32(val5))
        
    if event == '-SL6-': # SERVO SPEED
        val6 = int(values['-SL6-'])
        # window['-TXT6-'].update(val6)
        pub_val6.publish(Float32(val6))
    if event == '-Z6-' or reset_all:
        val6 = 150
        window['-SL6-'].update(val6)
        # window['-TXT6-'].update(val6)
        pub_val6.publish(Float32(val6))
        
    if event == '-SL7-':
        val7 = int(values['-SL7-'])
        # window['-TXT7-'].update(val7)
        pub_val7.publish(Float32(val7))
    if event == '-Z7-' or reset_all:
        val7 = 0
        window['-SL7-'].update(val7)
        # window['-TXT7-'].update(val7)
        pub_val7.publish(Float32(val7))
        
    if event == '-SL8-':
        val8 = int(values['-SL8-'])
        # window['-TXT8-'].update(val8)
        pub_val8.publish(Float32(val8))
    if event == '-Z8-' or reset_all:
        val8 = 0
        window['-SL8-'].update(val8)
        # window['-TXT8-'].update(val8)
        pub_val8.publish(Float32(val8))

    if event == '-SL9-':
        val9 = int(values['-SL9-'])
        # window['-TXT9-'].update(val9)
        pub_val9.publish(Float32(val9))
    if event == '-Z9-' or reset_all:
        val9 = 0
        window['-SL9-'].update(val9)
        # window['-TXT9-'].update(val9)
        pub_val9.publish(Float32(val9))

    if event == '-SL10-':
        val10 = int(values['-SL10-'])
        # window['-TXT10-'].update(val10)
        pub_val10.publish(Float32(val10))
    if event == '-Z10-' or reset_all:
        val10 = 0
        window['-SL10-'].update(val10)
        # window['-TXT10-'].update(val10)
        pub_val10.publish(Float32(val10))
        
    if event == '-SL11-':
        val11 = int(values['-SL11-'])
        # window['-TXT11-'].update(val11)
        pub_val11.publish(Float32(val11))
    if event == '-Z11-' or reset_all:
        val11 = 0
        window['-SL11-'].update(val11)
        # window['-TXT11-'].update(val11)
        pub_val11.publish(Float32(val11))
        
    if event == '-SL12-':
        val12 = int(values['-SL12-'])
        # window['-TXT12-'].update(val12)
        pub_val12.publish(Float32(val12))
    if event == '-Z12-' or reset_all:
        val12 = 0
        window['-SL12-'].update(val12)
        # window['-TXT12-'].update(val12)
        pub_val12.publish(Float32(val12))

    # Handle Mode buttons
    if event == '-POSE1-':
        pub_mode.publish(Int32(1))
    if event == '-POSE2-':
        pub_mode.publish(Int32(2))
    if event == '-POSE3-':
        pub_mode.publish(Int32(3))
    if event == '-POSE4-':
        pub_mode.publish(Int32(4))
        
    if event == '-STOP-':
        pub_mode.publish(Int32(5)) # set the mode to stop

        # Publish all values when Stop set, in case the contol app was restarted.
        pub_val1.publish(Float32(val1))
        pub_val2.publish(Float32(val2))
        pub_val3.publish(Float32(val3))
        pub_val4.publish(Float32(val4))
        pub_val5.publish(Float32(val5))
        pub_val6.publish(Float32(val6))
        pub_val7.publish(Float32(val7))
        pub_val8.publish(Float32(val8))
        pub_val9.publish(Float32(val9))
        pub_val10.publish(Float32(val10))
        pub_val11.publish(Float32(val11))
        pub_val12.publish(Float32(val12))


    if event == '-WALK-':
        pub_mode.publish(Int32(6))
        
        
        
    
        
        
window.close()

