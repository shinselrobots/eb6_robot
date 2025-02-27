#!/usr/bin/env python3

import rospy
import logging
from eb_servos.srv import ReturnJointStates
import time
import math
#import sys
#import signal

from std_msgs.msg import Int32
#from std_msgs.msg import Bool
#from geometry_msgs.msg import Point32
from std_msgs.msg import Float32
#from sensor_msgs.msg import Joy

#from dynamixel_msgs.msg import JointState # single servo messages
#from dynamixel_msgs.msg import JointStateArray
from servo_joint_list import *

import PySimpleGUI as psg


# ROS
rospy.init_node('Servo_Monitor_GUI')


# GUI Layout
psg.set_options(font=('Arial Bold', 12))
# psg.theme('Dark Green 5')

# ======================================================================================================
layout = [
    [
        psg.Button("B0", enable_events=True, key='-B0-'),
        psg.Button("B1", enable_events=True, key='-B1-'),
        psg.Button("B2", enable_events=True, key='-B2-'),
        psg.Button("B3", enable_events=True, key='-B3-'),
    ],
    
    [   # Header Row
        #psg.Text('Position Load Temperature', size=(50, 1), expand_x=False, justification='right',
            # font=('Arial Bold', 12), relief="raised", border_width=5, expand_y= True,
        #), 
        
        psg.Text('  Joint Name', size=(20, 1), expand_x=False, justification='left',
        ),
        
        psg.Text('Position', size=(8, 1), expand_x=False, justification='center',
        ),
        
        psg.Text('Load', size=(8, 1), expand_x=False, justification='center', 
        ),
        
        psg.Text('Temp (C)', size=(8, 1), expand_x=False, justification='center',
        ),

        # Dummy place holder to align with side buttons
        #psg.Text('', size=(6, 1), expand_x=False, justification='center',
        #),
        
    ],

    [   # Servo 1  
        # Label 
        psg.Text('x', key='-Name1-', size=(20, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
            # font=('Arial Bold', 12), relief="raised", border_width=5, expand_y= True,
        ),
        # Position
        psg.Text('0.00', key='-Pos1-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        # Load
        psg.Text('00.0', key='-Load1-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        # Temp
        psg.Text('00.0', key='-Temp1-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        # psg.Button("RST", enable_events=True, key='-Rst1-'),
    ],
    
]

# ======================================================================================================

# INIT
window = psg.Window('Robot Walk Parameters', layout, size=(715, 600), element_justification='c')
val1 = 0
val2 = 0
val3 = 0
val4 = 0

# LOOP
while True:
    print("")
    print("---------------------------")


    event, values = window.read(timeout=0)
    # print(event, values)
    
    if event == psg.WIN_CLOSED or event == 'Exit':
        break
        
    #if event == '-Rst1-':
    #    val1 = val1 + 1
    #    window['-Pos1-'].update(val1)


    joint_names = all_servo_joints

    rospy.wait_for_service("return_joint_states")
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException as e:
        print("error when calling return_joint_states: %s" %e)
        sys.exit(1)

    name_txt = ''
    position_txt = ''
    effort_txt = ''
    temp_txt = ''
           
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
        
            print("joint %s not found!"%joint_name)
        else:
            flag_string = ''
            if math.fabs(resp.effort[ind]) > .1:
                flag_string = ' <----'
            elif resp.effort[ind] == 0.0:
                flag_string = ' <<<<<'

            if(ind == 6 or ind == 7 or ind == 11):
                print("")
                
            #print("%s = \t %-2.3f   %s" % (joint_name, resp.effort[ind], flag_string)) 
            print('{0:7d} {1:30s} {2: 2.3f} {3: 2.2f} {4: 2.1f} {5:s}'.format(
                ind+1, joint_name, resp.position[ind], resp.effort[ind], resp.temp[ind], flag_string))
             
            #print(f"{joint_name} = \t {resp.effort[ind]}")
            temperature_color = "white"
            effort_color = "white"
            if math.fabs(resp.position[ind]) > .5:
                temperature_color = "dark red"
            if math.fabs(resp.position[ind]) > .4:
                effort_color = "dark red"
       
            if ind == 0:
                window['-Name1-'].update(joint_name)
                window['-Pos1-' ].update("{:.3f}".format(resp.position[ind]))
                window['-Load1-'].update("{:.2f}".format(resp.effort[ind]), text_color = effort_color)
                window['-Temp1-'].update("{:.1f}".format(resp.temp[ind]), text_color = temperature_color)
    

    rospy.sleep(0.1)   
        
window.close()

