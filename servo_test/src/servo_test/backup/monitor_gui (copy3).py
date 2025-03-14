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
from eb_servos.set_pose import *    # for RobotPose

import PySimpleGUI as psg


# ROS
rospy.init_node('Servo_Monitor_GUI')
pub_pose = rospy.Publisher('/walk_mode', Int32, queue_size=1)
robotpose = RobotPose("monitor_gui")

# GUI Layout
psg.set_options(font=('Arial Bold', 12))
# psg.theme('Dark Green 5')

# ======================================================================================================
layout = [

    [
        psg.Text('Set Pose: ', size=(11, 1), expand_x=False, justification='left', ),
        psg.Button("0", enable_events=True, key='-POSE0-'),
        psg.Button("1", enable_events=True, key='-POSE1-'),
        psg.Button("2", enable_events=True, key='-POSE2-'),
        psg.Button("3", enable_events=True, key='-POSE3-'),
        psg.Button("4", enable_events=True, key='-POSE4-'),
        psg.Button("5", enable_events=True, key='-POSE5-'),
    ],
    
    
    [   # Header Row
        #psg.Text('Position Load Temperature', size=(50, 1), expand_x=False, justification='right',
            # font=('Arial Bold', 12), relief="raised", border_width=5, expand_y= True,
        #), 
        
        psg.Text('  Joint Name', size=(23, 1), expand_x=False, justification='left',
        ),
        
        psg.Text('Position', size=(8, 1), expand_x=False, justification='center',
        ),
        
        psg.Text('Load', size=(8, 1), expand_x=False, justification='center', 
        ),
        
        psg.Text('Temp', size=(8, 1), expand_x=False, justification='center',
        ),

        # Dummy place holder to align with side buttons
        #psg.Text('', size=(6, 1), expand_x=False, justification='center',
        #),
        
    ],

    [   # Servo 1  
        # Label 
        psg.Text('x', key='-Name1-', size=(23, 1), enable_events=True, expand_x=False, 
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
    
    [   # Servo 2  
        psg.Text('x', key='-Name2-', size=(23, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos2-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load2-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp2-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
    ],
    
    [   # Servo 3  
        psg.Text('x', key='-Name3-', size=(23, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos3-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load3-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp3-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
    ],
    
    [   # Servo 4  
        psg.Text('x', key='-Name4-', size=(23, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos4-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load4-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp4-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
    ],
    
    [   # Servo 5  
        psg.Text('x', key='-Name5-', size=(23, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos5-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load5-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp5-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
    ],
    
    [   # Servo 6  
        psg.Text('x', key='-Name6-', size=(23, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos6-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load6-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp6-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
    ],
    
    [   # Skip a row
        psg.Text(' ', size=(1, 1),  expand_x=False,  justification='left', font=('Arial', 2),
        ),
    ],
    
    [   # Servo 7  
        psg.Text('x', key='-Name7-', size=(23, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos7-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load7-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp7-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
    ],
    
    [   # Servo 8  
        psg.Text('x', key='-Name8-', size=(23, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos8-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load8-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp8-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
    ],
    
    [   # Servo 9  
        psg.Text('x', key='-Name9-', size=(23, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos9-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load9-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp9-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
    ],
    
    [   # Servo 10  
        psg.Text('x', key='-Name10-', size=(23, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos10-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load10-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp10-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
    ],

    [   # Skip a row
        psg.Text(' ', size=(1, 1),  expand_x=False,  justification='left', font=('Arial', 2),
        ),
    ],
    
    [   # Servo 11  
        psg.Text('x', key='-Name11-', size=(23, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos11-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load11-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp11-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
    ],
    
    [   # Servo 12  
        psg.Text('x', key='-Name12-', size=(23, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos12-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load12-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp12-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
    ],
    
    [   # Servo 13  
        psg.Text('x', key='-Name13-', size=(23, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos13-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load13-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp13-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
    ],
    
    [   # Servo 14  
        psg.Text('x', key='-Name14-', size=(23, 1), enable_events=True, expand_x=False, 
            justification='left', relief="raised",
        ),
        psg.Text('0.00', key='-Pos14-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ),
        psg.Text('00.0', key='-Load14-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
        psg.Text('00.0', key='-Temp14-', size=(8, 1), enable_events=True, relief="raised",
            expand_x=False, justification='center',
        ), 
    ],
    
    
]

# ======================================================================================================

# INIT
window = psg.Window('Robot Walk Parameters', layout, size=(520, 600), element_justification='c')
val1 = 0
val2 = 0
val3 = 0
val4 = 0

# LOOP
while True:
    #print("")
    #print("---------------------------")


    event, values = window.read(timeout=0)
    # print(event, values)
    
    if event == psg.WIN_CLOSED or event == 'Exit':
        break
        
    if event == '-POSE0-':
        robotpose.move(0, 0.3) # Pose, Speed
        
    if event == '-POSE1-':
        robotpose.move(1, 0.3) # Pose, Speed
        
    if event == '-POSE2-':
        robotpose.move(2, 0.3) # Pose, Speed
        
    if event == '-POSE3-':
        robotpose.move(3, 0.3) # Pose, Speed
        
    if event == '-POSE4-':
        robotpose.move(4, 0.3) # Pose, Speed

    if event == '-POSE5-':
        robotpose.move(5, 0.3) # Pose, Speed
        
        


    joint_names = all_servo_joints

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

            temp_color = psg.theme_background_color() # "white"
            effort_color = psg.theme_background_color() #"white"
            if math.fabs(resp.temp[ind]) > 50.0:
                temp_color = "red"
            if math.fabs(resp.effort[ind]) > .1:
                effort_color = "red"
                
            servo_number = ind +1
            error_str = ''
            
            if servo_number == 1:
                window['-Name1-'].update(joint_name+error_str)
                window['-Pos1-' ].update("{:.3f}".format(resp.position[ind]))
                window['-Load1-'].update("{:.2f}".format(resp.effort[ind]), background_color = effort_color)
                window['-Temp1-'].update("{:.1f}".format(resp.temp[ind]), background_color = temp_color)
    
            elif servo_number == 2:
                window['-Name2-'].update(joint_name+error_str)
                window['-Pos2-' ].update("{:.3f}".format(resp.position[ind]))
                window['-Load2-'].update("{:.2f}".format(resp.effort[ind]), background_color = effort_color)
                window['-Temp2-'].update("{:.1f}".format(resp.temp[ind]), background_color = temp_color)
    
            elif servo_number == 3:
                window['-Name3-'].update(joint_name+error_str)
                window['-Pos3-' ].update("{:.3f}".format(resp.position[ind]))
                window['-Load3-'].update("{:.2f}".format(resp.effort[ind]), background_color = effort_color)
                window['-Temp3-'].update("{:.1f}".format(resp.temp[ind]), background_color = temp_color)
    
            elif servo_number == 4:
                window['-Name4-'].update(joint_name+error_str)
                window['-Pos4-' ].update("{:.3f}".format(resp.position[ind]))
                window['-Load4-'].update("{:.2f}".format(resp.effort[ind]), background_color = effort_color)
                window['-Temp4-'].update("{:.1f}".format(resp.temp[ind]), background_color = temp_color)
    
            elif servo_number == 5:
                window['-Name5-'].update(joint_name+error_str)
                window['-Pos5-' ].update("{:.3f}".format(resp.position[ind]))
                window['-Load5-'].update("{:.2f}".format(resp.effort[ind]), background_color = effort_color)
                window['-Temp5-'].update("{:.1f}".format(resp.temp[ind]), background_color = temp_color)
    
            elif servo_number == 6:
                window['-Name6-'].update(joint_name+error_str)
                window['-Pos6-' ].update("{:.3f}".format(resp.position[ind]))
                window['-Load6-'].update("{:.2f}".format(resp.effort[ind]), background_color = effort_color)
                window['-Temp6-'].update("{:.1f}".format(resp.temp[ind]), background_color = temp_color)
    
            elif servo_number == 7:
                window['-Name7-'].update(joint_name+error_str)
                window['-Pos7-' ].update("{:.3f}".format(resp.position[ind]))
                window['-Load7-'].update("{:.2f}".format(resp.effort[ind]), background_color = effort_color)
                window['-Temp7-'].update("{:.1f}".format(resp.temp[ind]), background_color = temp_color)
    
            elif servo_number == 8:
                window['-Name8-'].update(joint_name+error_str)
                window['-Pos8-' ].update("{:.3f}".format(resp.position[ind]))
                window['-Load8-'].update("{:.2f}".format(resp.effort[ind]), background_color = effort_color)
                window['-Temp8-'].update("{:.1f}".format(resp.temp[ind]), background_color = temp_color)
    
            elif servo_number == 9:
                window['-Name9-'].update(joint_name+error_str)
                window['-Pos9-' ].update("{:.3f}".format(resp.position[ind]))
                window['-Load9-'].update("{:.2f}".format(resp.effort[ind]), background_color = effort_color)
                window['-Temp9-'].update("{:.1f}".format(resp.temp[ind]), background_color = temp_color)
    
            elif servo_number == 10:
                window['-Name10-'].update(joint_name+error_str)
                window['-Pos10-' ].update("{:.3f}".format(resp.position[ind]))
                window['-Load10-'].update("{:.2f}".format(resp.effort[ind]), background_color = effort_color)
                window['-Temp10-'].update("{:.1f}".format(resp.temp[ind]), background_color = temp_color)
    
            elif servo_number == 11:
                window['-Name11-'].update(joint_name+error_str)
                window['-Pos11-' ].update("{:.3f}".format(resp.position[ind]))
                window['-Load11-'].update("{:.2f}".format(resp.effort[ind]), background_color = effort_color)
                window['-Temp11-'].update("{:.1f}".format(resp.temp[ind]), background_color = temp_color)
    
            elif servo_number == 12:
                window['-Name12-'].update(joint_name+error_str)
                window['-Pos12-' ].update("{:.3f}".format(resp.position[ind]))
                window['-Load12-'].update("{:.2f}".format(resp.effort[ind]), background_color = effort_color)
                window['-Temp12-'].update("{:.1f}".format(resp.temp[ind]), background_color = temp_color)
    
            elif servo_number == 13:
                window['-Name13-'].update(joint_name+error_str)
                window['-Pos13-' ].update("{:.3f}".format(resp.position[ind]))
                window['-Load13-'].update("{:.2f}".format(resp.effort[ind]), background_color = effort_color)
                window['-Temp13-'].update("{:.1f}".format(resp.temp[ind]), background_color = temp_color)
    
            elif servo_number == 14:
                window['-Name14-'].update(joint_name+error_str)
                window['-Pos14-' ].update("{:.3f}".format(resp.position[ind]))
                window['-Load14-'].update("{:.2f}".format(resp.effort[ind]), background_color = effort_color)
                window['-Temp14-'].update("{:.1f}".format(resp.temp[ind]), background_color = temp_color)
    

    rospy.sleep(0.1)   
        
window.close()

