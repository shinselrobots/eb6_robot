#!/usr/bin/env python3

import rospy
import logging
from eb_servos.srv import ReturnJointStates
import time
import math
#import sys
#import signal

from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Int32
#from std_msgs.msg import Bool
#from geometry_msgs.msg import Point32
from std_msgs.msg import Float32
#from sensor_msgs.msg import Joy

#from dynamixel_msgs.msg import JointState # single servo messages
#from dynamixel_msgs.msg import JointStateArray
from servo_joint_list import *
from eb_servos.set_pose import *    # for RobotPose
from eb_servos.set_servo_torque import *

#import PySimpleGUI as psg
import FreeSimpleGUI as psg


import actionlib
import actionlib.action_client
from behavior_msgs.msg import CommandState 

# ROS
rospy.init_node('servo_dashboard')
#pub_pose = rospy.Publisher('/walk_mode', Int32, queue_size=1)
robotpose = RobotPose("servo_dashboard")

# GUI Layout
psg.set_options(font=('Arial Bold', 10))
# psg.theme('Dark Green 5')

#        psg.Text('Set Pose: ', size=(11, 1), expand_x=False, justification='left', ),
#         psg.Text('Volts', size=(7, 1), expand_x=False, font=('Arial Bold', 12), pad=(0,0), ),

# pad=(0,0),
# ======================================================================================================
layout = [

    [
        psg.Text('No Battery', enable_events=True, key='-BATTERY-', size=(11, 1),
            expand_x=False, justification='center', font=('Arial Bold', 12), text_color='white',  ), 

        psg.Button("0", enable_events=True, key='-POSE0-'),
        psg.Button("1", enable_events=True, key='-POSE1-'),
        psg.Button("2", enable_events=True, key='-POSE2-'),
        psg.Button("3", enable_events=True, key='-POSE3-'),
        psg.Button("4", enable_events=True, key='-POSE4-'),
        psg.Button("5", enable_events=True, key='-POSE5-'),
    ],

    [
        psg.Text('', enable_events=False, size=(10, 1),
            expand_x=False, justification='center', font=('Arial Bold', 10), ), 
    ],
    
    [   # Lights
        psg.Button("Eyes On",  enable_events=True, size=(4, 2), key='-EYES_ON-', ),
        psg.Button("Ears On",  enable_events=True, size=(4, 2), key='-EARS_ON-'),
        psg.Button("Body On", enable_events=True, size=(4, 2), key='-BODY_ON-'),

        psg.Button("Eyes Off", enable_events=True, size=(4, 2), key='-EYES_OFF-'),
        psg.Button("Ears Off", enable_events=True, size=(4, 2), key='-EARS_OFF-'),
        psg.Button("Body Off", enable_events=True, size=(4, 2), key='-BODY_OFF-'),
    ],

    [   # Behaviors
        psg.Button("Head Zero",  enable_events=True, size=(7, 2), key='-HEAD_CENTER-', ),
        psg.Button("Wake",       enable_events=True, size=(7, 2), key='-WAKE-'),
        psg.Button("Sleep",      enable_events=True, size=(7, 2), key='-SLEEP-'),
        psg.Button("Tell Joke",  enable_events=True, size=(7, 2), key='-JOKE-'),
    ],
    
    [   # Behaviors
        psg.Button("Follow", enable_events=True, size=(7, 2), key='-FOLLOW-', ),
        psg.Button("Danger", enable_events=True, size=(7, 2), key='-DANGER-'),
        psg.Button("      ", enable_events=True, size=(7, 2), key='-TBD1-'),
        psg.Button("      ", enable_events=True, size=(7, 2), key='-TBD2-'),
    ],
    
    [
        psg.Text('', enable_events=False, size=(10, 1),
            expand_x=False, justification='center', font=('Arial Bold', 10), ), 
    ],

    
    [   # Servo Status header row
        psg.Text('Joint Name', size=(19, 1), expand_x=False, justification='center', font=('Arial Bold', 11),
        ),
        psg.Text('Position', size=(8, 1), expand_x=False, justification='left', font=('Arial Bold', 11),
        ),
        psg.Text('Load', size=(5, 1), expand_x=False, justification='left',  font=('Arial Bold', 11),
        ),
        psg.Text('Temp', size=(6, 1), expand_x=False, justification='left', font=('Arial Bold', 11),
        ),
        psg.Checkbox("", key='-Torq_All-', enable_events=True, default=True), 
    ],

    [   # Servo 1  
        # Label 
        psg.Text('x', key='-Name1-', size=(22, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
            # font=('Arial Bold', 12), relief="raised", border_width=5, expand_y= True,
        ),
        # Position
        psg.Text('0.00', key='-Pos1-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        # Load
        psg.Text('00.0', key='-Load1-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        # Temp
        psg.Text('00.0', key='-Temp1-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Checkbox("", key='-Torq1-', enable_events=True, default=True), 
    ],
    
    [   # Servo 2  
        psg.Text('x', key='-Name2-', size=(22, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised", 
        ),
        psg.Text('0.00', key='-Pos2-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load2-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp2-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Checkbox("", key='-Torq2-', enable_events=True, default=True), 
    ],
    
    [   # Servo 3  
        psg.Text('x', key='-Name3-', size=(22, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos3-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load3-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp3-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Checkbox("", key='-Torq3-', enable_events=True, default=True), 
    ],
    
    [   # Servo 4  
        psg.Text('x', key='-Name4-', size=(22, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos4-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load4-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp4-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Checkbox("", key='-Torq4-', enable_events=True, default=True), 
    ],
    
    [   # Servo 5  
        psg.Text('x', key='-Name5-', size=(22, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos5-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load5-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp5-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Checkbox("", key='-Torq5-', enable_events=True, default=True), 
    ],
    
    [   # Servo 6  
        psg.Text('x', key='-Name6-', size=(22, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos6-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load6-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp6-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Checkbox("", key='-Torq6-', enable_events=True, default=True), 
    ],
    
    [   # Skip a row
        psg.Text(' ', size=(1, 1),  expand_x=False,  justification='left', font=('Arial', 2),
        ),
    ],
    
    [   # Servo 7  
        psg.Text('x', key='-Name7-', size=(22, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos7-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load7-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp7-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Checkbox("", key='-Torq7-', enable_events=True, default=True), 
    ],
    
    [   # Servo 8  
        psg.Text('x', key='-Name8-', size=(22, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos8-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load8-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp8-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Checkbox("", key='-Torq8-', enable_events=True, default=True), 
    ],
    
    [   # Servo 9  
        psg.Text('x', key='-Name9-', size=(22, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos9-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load9-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp9-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Checkbox("", key='-Torq9-', enable_events=True, default=True), 
    ],
    
    [   # Skip a row
        psg.Text(' ', size=(1, 1),  expand_x=False,  justification='left', font=('Arial', 2),
        ),
    ],
    
    [   # Servo 10  
        psg.Text('x', key='-Name10-', size=(22, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos10-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load10-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp10-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Checkbox("", key='-Torq10-', enable_events=True, default=True), 
    ],

    [   # Servo 11  
        psg.Text('x', key='-Name11-', size=(22, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos11-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load11-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp11-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Checkbox("", key='-Torq11-', enable_events=True, default=True), 
    ],
    
    [   # Servo 12  
        psg.Text('x', key='-Name12-', size=(22, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos12-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load12-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp12-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Checkbox("", key='-Torq12-', enable_events=True, default=True), 
    ],
    
    [   # Skip a row
        psg.Text(' ', size=(1, 1),  expand_x=False,  justification='left', font=('Arial', 2),
        ),
    ],
    
    [   # Servo 13  
        psg.Text('x', key='-Name13-', size=(22, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos13-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load13-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp13-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Checkbox("", key='-Torq13-', enable_events=True, default=True), 
    ],
    
    [   # Servo 14  
        psg.Text('x', key='-Name14-', size=(22, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos14-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load14-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp14-', size=(7, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Checkbox("", key='-Torq14-', enable_events=True, default=True), 
    ],
    
    
]

# ======================================================================================================

# INIT
window = psg.Window('EB Robot Servo Dashboard', layout, size=(450, 750), 
    element_justification='c', enable_close_attempted_event=True)
torq_cb_event = [False for i in range(14)]
battery_update = False
battery_voltage_str = "0.0 Volts"
battery_background_color = psg.theme_background_color()
battery_text_color = "white"

# Publishers
behavior_cmd_pub = rospy.Publisher('behavior/cmd', CommandState, queue_size=2)
pub_ear_cmd =   rospy.Publisher('/head/ear_cmd',   UInt16, queue_size=10)        
pub_eye_cmd =   rospy.Publisher('/head/eye_cmd',   UInt16, queue_size=10)        
pub_eye_color = rospy.Publisher('/head/eye_color', UInt32, queue_size=10)        
pub_body_strip_color = rospy.Publisher('/body/strip_color', UInt32, queue_size=8)

def handle_torque_cb(index, window_name):
    if torq_cb_event[index]:
        torq_cb_event[index] = False # reset event trigger
        if window[window_name].get():
            SetSingleServoTorque(1.0, joint_name)
            window[window_name].update(checkbox_color=psg.theme_background_color())
        else:
            SetSingleServoTorque(0.0, joint_name)
            window[window_name].update(checkbox_color='red')

def handle_window_update(window_name, window_pos, window_load, window_temp):
    window[window_name].update(servo_number_str + ': ' + joint_name_trim + error_str)
    window[window_pos].update("{:.3f}".format(resp.position[ind]))
    window[window_load].update("{:.2f}".format(resp.effort[ind]), background_color = effort_color)
    window[window_temp].update("{:.1f}".format(resp.temp[ind]), background_color = temp_color)

def send_behavior_command(command, param1='', param2=''):
    print("sending behavior command: %s parm1: %s parm2: %s" % (command, param1, param2))

    msg = CommandState()
    msg.commandState = command
    msg.param1 = param1
    msg.param2 = param2
    behavior_cmd_pub.publish(msg)

    print("done.")


def battery_callback(data):
    global battery_update
    global battery_voltage_str
    global battery_background_color
    global battery_text_color
    
    # Also lights bottom 4 body LEDs to indicate voltage level
    
    battery_voltage = data.data
    volts_str = "%2.1f" % battery_voltage
    battery_voltage_str = volts_str + " Volts"
    battery_background_color = "green"
    battery_text_color = "white"
    if battery_voltage < 10.00:   # No battery 
        battery_voltage_str = "No Battery"
        battery_background_color = psg.theme_background_color() # "light gray"

    elif battery_voltage <= 12.5:   # 2 min 
        battery_voltage_str = volts_str + "v 2min!"
        battery_background_color = "red" 

    elif battery_voltage <= 14.0:   # 5 min 
        battery_voltage_str = volts_str + "v <5min"
        battery_background_color = "red"

    elif battery_voltage <= 14.25:    # 10 min
        battery_voltage_str = volts_str + "v <10min"
        battery_background_color = "orange"
        battery_text_color = "black"

    elif battery_voltage <= 14.5:    # 20 min
        battery_voltage_str = volts_str + "v <20min"
        battery_background_color = "orange"
        battery_text_color = "black"

    elif battery_voltage <= 15.0:    # 30 min
        battery_voltage_str = volts_str + "v <30min"
        battery_background_color = "yellow"
        battery_text_color = "black"

    elif battery_voltage <= 15.25:    # 60 min
        battery_voltage_str = volts_str + "v <60min"
        battery_background_color = "blue"

    battery_update = True
    
#    window['-BATTERY-'].update(battery_voltage_str, text_color=battery_text_color)
#event, values = window.read() # must do before callback fires
battery_sub = rospy.Subscriber("/battery_voltage", Float32, battery_callback)
#battery_sub = rospy.Subscriber("/battery_raw", UInt16, battery_callback)

#text_color=battery_text_color,
# LOOP
while True:

    if battery_update:
        window['-BATTERY-'].update(battery_voltage_str, background_color = battery_background_color, text_color = battery_text_color)
        battery_update = False
        

    event, values = window.read(timeout=0)


    
    #if event == psg.WINDOW_CLOSE_ATTEMPTED_EVENT:
    #    pass

    if event == psg.WIN_CLOSED or event == 'Exit':
        break

    elif event == '-POSE0-':
        window['-Torq_All-'].update(value=True)      
        robotpose.move(0, 0.3) # Pose, Speed
        event = '-Torq_All-' # update gui
        
    elif event == '-POSE1-':
        window['-Torq_All-'].update(value=True)
        robotpose.move(1, 0.3) # Pose, Speed
        event = '-Torq_All-' # update gui
        
    elif event == '-POSE2-':
        window['-Torq_All-'].update(value=True)
        robotpose.move(2, 0.3) # Pose, Speed
        event = '-Torq_All-' # update gui
        
    elif event == '-POSE3-':
        window['-Torq_All-'].update(value=True)
        start_time = time.time()
        print()        
        print()        
        print()        
        print("****************************************************************************")
        robotpose.move(3, 0.3) # Pose, Speed
        end_time = time.time()
        elapsed_time = end_time - start_time
        print()        
        print()        
        print()        
        print("Time: {0: 8.4f} ".format(elapsed_time))

        print("****************************************************************************")
        print()        
        print()        
        print()        
        event = '-Torq_All-' # update gui
        
    elif event == '-POSE4-':
        window['-Torq_All-'].update(value=True)
        robotpose.move(4, 0.3) # Pose, Speed
        event = '-Torq_All-' # update gui

    elif event == '-POSE5-':
        window['-Torq_All-'].update(value=True)
        robotpose.move(5, 0.3) # Pose, Speed
        event = '-Torq_All-' # update gui


    # Lights
    elif event == '-EYES_ON-':
        pub_eye_cmd.publish(2) # 2 = Turn eyes on, normal blink mode
    elif event == '-EARS_ON-':
        pub_ear_cmd.publish(3) # 3 = Turn ear lights on, rainbow mode
    elif event == '-BODY_ON-':
        pass # Use eb_sysmon for this

    elif event == '-EYES_OFF-':
        pub_eye_cmd.publish(0) # Turn eyes off
    elif event == '-EARS_OFF-':
        pub_ear_cmd.publish(0) # Turn ears off
    elif event == '-BODY_OFF-':
        self.pub_body_strip_color.publish(0xff000000) # force all LEDS off


    # Behaviors
    # see sheldon speech recognition/config/yaml and intent_extraction.py for examples
    elif event == '-HEAD_CENTER-':
        print("TODO - HEAD_CENTER not implemented!")
        
    elif event == '-WAKE-':
        send_behavior_command('WAKE', 'x', 'x')
        
    elif event == '-SLEEP-':
        send_behavior_command('SLEEP', 'x', 'x')
        
    elif event == '-JOKE-':
        send_behavior_command('PY_TELL_A_JOKE', '1', 'x')
        
    elif event == '-FOLLOW-':
        send_behavior_command('FOLLOW_ME')
        
    elif event == '-DANGER-':
        send_behavior_command('-DANGER-')
        
    elif event == '-TBD1-':
        send_behavior_command('-TBD1-')
        
    elif event == '-TBD2-':
        send_behavior_command('-TBD2-')
 
 
 
        
        
    if event == '-Torq_All-':
        torq_cb_event = [True for i in range(14)]
        if window['-Torq_All-'].get():
            print("CHECKBOX ALL TRUE")
            #SetServoTorque(1.0, all_servo_joints_and_wheels)
            window['-Torq1-'].update(value=True)            
            window['-Torq2-'].update(value=True)            
            window['-Torq3-'].update(value=True)            
            window['-Torq4-'].update(value=True)            
            window['-Torq5-'].update(value=True)            
            window['-Torq6-'].update(value=True)            
            window['-Torq7-'].update(value=True)            
            window['-Torq8-'].update(value=True)            
            window['-Torq9-'].update(value=True)            
            window['-Torq10-'].update(value=True)            
            window['-Torq11-'].update(value=True)            
            window['-Torq12-'].update(value=True)            
            window['-Torq13-'].update(value=True)            
            window['-Torq14-'].update(value=True)            
            window['-Torq_All-'].update(checkbox_color=psg.theme_background_color())
        else:
            print("CHECKBOX ALL FALSE")
            #SetServoTorque(0.0, all_servo_joints_and_wheels)
            window['-Torq1-'].update(value=False)            
            window['-Torq2-'].update(value=False)            
            window['-Torq3-'].update(value=False)            
            window['-Torq4-'].update(value=False)            
            window['-Torq5-'].update(value=False)            
            window['-Torq6-'].update(value=False)            
            window['-Torq7-'].update(value=False)            
            window['-Torq8-'].update(value=False)            
            window['-Torq9-'].update(value=False)            
            window['-Torq10-'].update(value=False)            
            window['-Torq11-'].update(value=False)            
            window['-Torq12-'].update(value=False)            
            window['-Torq13-'].update(value=False)            
            window['-Torq14-'].update(value=False)            
            window['-Torq_All-'].update(checkbox_color='red')

    elif event == '-Torq1-':
        torq_cb_event[0] = True
    elif event == '-Torq2-':
        torq_cb_event[1] = True
    elif event == '-Torq3-':
        torq_cb_event[2] = True
    elif event == '-Torq4-':
        torq_cb_event[3] = True
    elif event == '-Torq5-':
        torq_cb_event[4] = True
    elif event == '-Torq6-':
        torq_cb_event[5] = True
    elif event == '-Torq7-':
        torq_cb_event[6] = True
    elif event == '-Torq8-':
        torq_cb_event[7] = True
    elif event == '-Torq9-':
        torq_cb_event[8] = True
    elif event == '-Torq10-':
        torq_cb_event[9] = True
    elif event == '-Torq11-':
        torq_cb_event[10] = True
    elif event == '-Torq12-':
        torq_cb_event[11] = True
    elif event == '-Torq13-':
        torq_cb_event[12] = True
    elif event == '-Torq14-':
        torq_cb_event[13] = True

    joint_names = all_servo_joints_and_wheels

    rospy.wait_for_service("return_joint_states")
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException as e:
        print("error when calling return_joint_states: %s" %e)
        sys.exit(1)

    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):        
            print("joint %s not found!"%joint_name)
           
        else:
            flag_string = ''
            if math.fabs(resp.effort[ind]) > .1:
                flag_string = ' <----'
            elif resp.effort[ind] == 0.0:
                flag_string = ' <<<<<'

            temp_color = psg.theme_background_color() 
            effort_color = psg.theme_background_color() 
            if math.fabs(resp.temp[ind]) > 50.0:
                temp_color = "red"
            if math.fabs(resp.effort[ind]) > .1:
                effort_color = "red"
                
            servo_number = ind +1
            servo_number_str = str(servo_number)
            error_str = ''
            joint_name_trim = joint_name[:-6]
            
            if servo_number == 1:
                handle_window_update( '-Name1-', '-Pos1-', '-Load1-', '-Temp1-')                
                handle_torque_cb(ind, '-Torq1-')    
            elif servo_number == 2:
                handle_window_update('-Name2-', '-Pos2-', '-Load2-', '-Temp2-')                
                handle_torque_cb(ind, '-Torq2-')                     
            elif servo_number == 3:
                handle_window_update('-Name3-', '-Pos3-', '-Load3-', '-Temp3-')                
                handle_torque_cb(ind, '-Torq3-')    
            elif servo_number == 4:
                handle_window_update('-Name4-', '-Pos4-', '-Load4-', '-Temp4-')                
                handle_torque_cb(ind, '-Torq4-')    
            elif servo_number == 5:
                handle_window_update('-Name5-', '-Pos5-', '-Load5-', '-Temp5-')                
                handle_torque_cb(ind, '-Torq5-')    
            elif servo_number == 6:
                handle_window_update('-Name6-', '-Pos6-', '-Load6-', '-Temp6-')                
                handle_torque_cb(ind, '-Torq6-')    
            elif servo_number == 7:
                handle_window_update('-Name7-', '-Pos7-', '-Load7-', '-Temp7-')                
                handle_torque_cb(ind, '-Torq7-')    
            elif servo_number == 8:
                handle_window_update('-Name8-', '-Pos8-', '-Load8-', '-Temp8-')                
                handle_torque_cb(ind, '-Torq8-')    
            elif servo_number == 9:
                handle_window_update('-Name9-', '-Pos9-', '-Load9-', '-Temp9-')                
                handle_torque_cb(ind, '-Torq9-')    
            elif servo_number == 10:
                handle_window_update('-Name10-', '-Pos10-', '-Load10-', '-Temp10-')                
                handle_torque_cb(ind, '-Torq10-')    
            elif servo_number == 11:
                handle_window_update('-Name11-', '-Pos11-', '-Load11-', '-Temp11-')                
                handle_torque_cb(ind, '-Torq11-')    
            elif servo_number == 12:
                handle_window_update('-Name12-', '-Pos12-', '-Load12-', '-Temp12-')                
                handle_torque_cb(ind, '-Torq12-')    
            elif servo_number == 13:
                handle_window_update('-Name13-', '-Pos13-', '-Load13-', '-Temp13-')                
                handle_torque_cb(ind, '-Torq13-')    
            elif servo_number == 14:
                handle_window_update('-Name14-', '-Pos14-', '-Load14-', '-Temp14-')                
                handle_torque_cb(ind, '-Torq14-')
    

    rospy.sleep(0.1)   
        
window.close()

