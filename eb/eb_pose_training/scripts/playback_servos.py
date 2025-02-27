#!/usr/bin/env python3
# Playback servo positions for scripting of robot behavior

# PARAMETERS:
# This script expects up to two parameters:
# 1: An optional filepath to the CSV file to execute
# 2: Number of times to repeat the CSV file. Default is zero (no repeat, just run once)
#    NOTE: if negative value submitted, single_step_mode is enabled, 
#          and playback waits for each step until it receives a button down from the joystick (Yellow Y button)
#          or receives a message:  rostopic pub -1 /user_mark_time std_msgs/UInt16  1


## I THINK this is only used for debugging (if I recall).
## Use play_script_behavior to run scripted bahaviors in demos, etc.

# NOTE! playback_servos and record_servos can be used at the same time!


import rospy
import logging
import time
import csv
import math
import sys
import signal

from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
# from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32

# for sound effects
import os 
# import thread
#from playsound import playsound
#from pygame import mixer
import pygame


# EB ONLY
#from dynamixel_controllers.srv import TorqueEnable, SetServoTorqueLimit, SetSpeed
from eb_servos.servo_joint_list import all_servo_joints, head_joints, right_leg_joints
from eb_servos.head_servo_publishers import *
from eb_servos.leg_servo_publishers import *

from eb_servos.standard_servo_positions import *
from eb_servos.set_servo_speed import *
from eb_servos.set_servo_torque import *
from eb_servos.srv import ReturnJointStates

from eb_servos.set_servo_pid import SetServoPGain, SetServoIGain, SetServoDGain

# Constants
# Turn State Machine
#turn_state_idle = 0
#turn_state_initializing = 1
#turn_state_turning = 2
#turn_state_ending = 3

interrupted = False

RAD_DEG = 57.296


def signal_handler(signal, frame):
    global interrupted
    interrupted = True


class PlaybackServoPositions():
    def __init__(self, csv_filepath, csv_loops):
        rospy.init_node('playback_servos')

        self.performance_time = rospy.get_rostime() # debug time to load and run the script
        self.step_start_time = self.performance_time

        # Use Python's CSV reader
        print('')
        if csv_filepath == None:
            print("Using Default CSV Filepath")
            csv_filepath = "/home/system/play_servos.csv" # default
        print("CSV FILE = ", csv_filepath)
        print('')
        csv_file = open(csv_filepath, "r") 
        #csv_file = open("/home/system/script_believer.csv", "r") 
        self.csv_reader = csv.DictReader(csv_file)
        self.csv_data = [row for row in self.csv_reader]

        performance_elapsed_time = rospy.Time.now() - self.performance_time
        elapsed_sec = performance_elapsed_time.to_sec()
        rospy.loginfo("PLAYBACK: DBG: Time to read CSV file = %04.2f seconds.",  elapsed_sec)

        
        self.csv_repeat_times = csv_loops
        self.single_step_mode = False    
        self.next_step = False
        self.servo_speed_base = 1.0 # default speed
        self.servo_speed_step = 0.3 # default speed

        if self.csv_repeat_times < 0:
            print("Single Step Mode Enabled!  Press Yellow Y Button to step")
            self.single_step_mode = True
            self.csv_repeat_times = 0

        # Save individual servo speed multiplier as array in ID order
        self.servo_names = all_servo_joints
        self.servo_speed_multiplier = []
        #self.servo_goal = []
        
        # initialize the arrays       
        for i in range(len(self.servo_names)):
            # print( str(i) + " :  [" + self.servo_names[i] + "]" )
            self.servo_speed_multiplier.append(1.0)
            #self.servo_goal.append(float('nan'))


            
        # PUBLISHERS
        # Publish wheel motor commands as low priority to motor control
        # NOT IMPLEMENTED ON EB ROBOT
        # Put this in launch file: 
        #   <!-- <remap from="cmd_vel" to="move_base/priority1"/>  --> 
        # self.pub_wheel_motors = rospy.Publisher('~cmd_vel', Twist, queue_size=5)
        # self.pub_wheel_motors = rospy.Publisher('move_base/priority1', Twist, queue_size=5)

        # Publish eye color changes
        self.pub_eye_color = rospy.Publisher('/head/eye_color', UInt32, queue_size=2)        

        # Publish robot light control
        self.pub_light_mode = rospy.Publisher('/body_led_mode', UInt16, queue_size=2)        

        # Publish microphone enable/disable by user (Note: user_enable vs system_enable)
        self.mic_user_enable_pub = rospy.Publisher('microphone/user_enable', Bool, queue_size=1)

        # enable/disable microphone when robot is talking or moving servos.  (Note system_enable vs. user_enable)
        self.mic_system_enable_pub = rospy.Publisher('/microphone/system_enable', Bool, queue_size=1)        


        # SUBSCRIBERS
        #compass_sub_ = nh_.subscribe<std_msgs::Float32>("/compass", 10, &WheelControl::compassMsgCallback, this);
        #imu_orientation_sub_ = nh_.subscribe<geometry_msgs::Point32>("/imu_orientation", 10, &WheelControl::imuOrientationMsgCallback, this);

        # subscribe to marker messages by user, to single step for debug
        time_mark_sub = rospy.Subscriber('/user_mark_time', UInt16, self.time_mark_cb) # arbitrary ID assigned by user

        # use joystick buttons for single step
        joystick_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)

        # get path to music file TODO
        self.using_music = False
        if self.using_music:
            self.resource_dir = rospy.get_param('resource_dir', 
              "/home/system/catkin_robot/src/eb/eb_behaviors/resources/sounds")
            self.music_file = os.path.join(self.resource_dir, "believer_trim2.wav")
            rospy.loginfo("DBG: music file: %s", self.music_file)
            pygame.init()


        rospy.loginfo("playback_servos initialized")


    # ===========================================================================
    # Utility functions

    def cleanup(self):
        # clean everyting up before exiting

        # stop any wheel motion
        #twist = Twist()
        #twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        #twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        #self.pub_wheel_motors.publish(twist)

        # un-mute the microphone
        self.mic_system_enable_pub.publish(True)

        # stop the music if it's still going
        if self.using_music:
            pygame.mixer.music.stop()
        rospy.loginfo("playback_servos Done")

    def time_mark_cb(self, msg):
        rospy.loginfo("got time ID marker from user")
        #id = msg.data
        self.next_step = True   
        
    def MoveServo(self, servo_goal_names, servo_goal_values, servo_name, publisher, value):
        # if servo was written into the file, set it, otherwise just return
        try:
            position = float(value)
            #rospy.loginfo("SET SERVO: Name = %s, Value = %02.4f", servo_name, position )
        except ValueError:
            rospy.loginfo("SET SERVO: Name = %s, NO VALUE for [%s]", servo_name, value )
            return

        if not math.isnan(position):
            # Send command to Servos
            #rospy.loginfo("PLAYBACK MoveServo: Setting servo %s, Value = %02.4f", servo_name, position )
            publisher.publish(position)
            # Save the name and value for the goal monitor
            joint_name = servo_name + '_joint'
            servo_goal_names.append(joint_name)
            servo_goal_values.append(position)


    def SetSServoSpeedMultiplier(self, servo_name, value):
        servo_name = servo_name + '_joint'
        # if servo was written into the file, set it, otherwise just return
        try:
            mul_speed = float(value)
            #rospy.loginfo("SET SERVO: Name = %s, Value = %02.4f", servo_name, speed )
        except ValueError:
            rospy.loginfo("SET SERVO MUL: Name = %s, NO VALUE for [%s]", servo_name, value )
            return
        if not math.isnan(mul_speed):
            # Save the multiplier. Global speeds are calculated from this.
            servo_index = self.servo_names.index(servo_name)
            self.servo_speed_multiplier[servo_index] = mul_speed

            # Set the speed
            final_speed = mul_speed * self.servo_speed_step * self.servo_speed_base
            rospy.loginfo("PLAYBACK ServoMul: Setting servo %s, Value = %02.4f", servo_name, final_speed )
            SetSingleServoSpeed(final_speed, servo_name)


    def SetPIDParam(self, pid_param, servo_name, str_value):
        servo_name = servo_name + '_joint'
        # if value was written into the file, set it, otherwise just return
        try:
            value = int(str_value)
            #rospy.loginfo("SET SERVO PID: Name = %s, Value = %d", servo_name, value )
        except ValueError:
            # Ignore NAN values. 
            # rospy.loginfo("SET SERVO PID: Name = %s, NO VALUE for [%s]", servo_name, str_value )
            return
        if not math.isnan(value):
            # Set the speed
            #rospy.loginfo("PLAYBACK SetPIDParam: Setting servo %s param %s to %d", servo_name, pid_param, value )
            joints = []
            joints.append(servo_name)
            try:
                if pid_param == 'p':
                    SetServoPGain(value, joints)
                elif pid_param == 'i':
                    SetServoIGain(value, joints)
                elif pid_param == 'd':
                    SetServoDGain(value, joints)
                else:
                    rospy.logwarn("SetPIDParam ERROR: PID Param not one of: p, i, d !") 
               
            except rospy.ROSInterruptException:
                rospy.loginfo("Oops! Exception occurred while trying to set single servo pid.") 


    def SetAllServos(self, value):
        SetServoSpeed(value, head_joints)
        SetServoSpeed(value, right_leg_joints)
        SetServoSpeed(value, left_leg_joints)

    def call_return_joint_states(self, joint_names):
        rospy.wait_for_service("return_joint_states")
        try:
            s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
            resp = s(joint_names)
        except rospy.ServiceException as e:
            rospy.logwarn("%s: Error when calling return_joint_states: %s" % ("playback_servos", e))
        # print("DBG: ************* RESULTS *****************")
        for (ind, joint_name) in enumerate(joint_names):
            if(not resp.found[ind]):
                rospy.logwarn("%s: joint %s not found!" % ("eb_pose_training", joint_name ))
            # else:
                #print("DBG: Joint found: %s, Position = %2.3f" % (joint_name, resp.position[ind]))
        return (resp.position, resp.velocity, resp.effort, resp.goal_pos, resp.error)

    def get_servo_positions(self, servo_names):
        # Single servo name, but service expects an array of names
        (position, velocity, effort, goal_pos, error) = self.call_return_joint_states(servo_names)
        return position # array of positions that match names


    #def Turn(self, turn_amount)
        #rospy.loginfo("PLAYBACK Beginning Turn for %3.1f degrees", turn_amount )
        # get starting angle
        #starting_angle = self.current_angle
        # set ending angle    
        # begin turn
        #self.turn_state = turn_state_initializing

    def wheelTurn(self, turn_speed): # positve = left
        # simple turn, does not keep track of distance!
        # uses script timing to control amount (kludge)
        #twist = Twist()
        #twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        #twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn_speed
        #self.pub_wheel_motors.publish(twist)
        rospy.logwarn("PLAYBACK: WHEEL TURN NOT IMPLEMENTED ON EB ROBOT!" )
        

    def wheelMove(self, move_speed):
        # simple move, does not keep track of distance!
        # uses script timing to control amount (kludge)
        #twist = Twist()
        #twist.linear.x = move_speed; twist.linear.y = 0; twist.linear.z = 0
        #twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        #self.pub_wheel_motors.publish(twist)
        rospy.logwarn("PLAYBACK: WHEEL MOVE NOT IMPLEMENTED ON EB ROBOT!" )

    # =============================== JOYSTICK CALLBACK =================================
    def joy_callback(self, data):
        # Joystick messages. 
        # Only one button handled in this file. Other buttons are handled by the record_servos program.

        if data.buttons[3] == 1:  # Yellow "Y" Button Pressed
            rospy.loginfo("Joystick Yellow Button Pressed. Moving to next Step!")
            self.next_step = True    
        

    # ===========================================================================
    # Run Script

    def run(self):

        # Initialize Servo settings
        rospy.loginfo("Setting initial ServoTorque")
        SetServoTorque(1.0, all_servo_joints)
        rospy.loginfo("Setting initial ServoSpeeds")
        self.SetAllServos(self.servo_speed_base * self.servo_speed_step)
        rospy.loginfo("Servo Speeds set")
        #SetSingleServoSpeed(1.5, 'right_leg_shoulder_rotate_joint')
        #SetSingleServoSpeed(1.5, 'left_leg_shoulder_rotate_joint')

        rospy.loginfo("======================================================")
        rospy.loginfo("               EXECUTING SCRIPT")
        rospy.loginfo("======================================================")

        if self.csv_repeat_times > 0:
            rospy.loginfo("CSV Repeat set to: %d", self.csv_repeat_times)


        # Initialize script settings
        self.script_start_time = rospy.get_rostime() # Entire Script time keeper
        self.script_row = 0
        #self.turn_state = turn_state_idle
        self.next_step = False
        start_time_offset = 0.0  # how long to wait before first action
        script_score_time = start_time_offset
        # mute the microphone, so the robot does not hear music and servos!
        self.mic_system_enable_pub.publish(False)
        if self.single_step_mode:
            print("***** SINGLE STEP MODE *****")
            print("Listening for Joystick button")


        # Number of times to play the script (for walking)
        for play_csv_loop in range(self.csv_repeat_times+1): # run at least once
            row = 0
            self.script_row = 0

            print("***** PLAYING CSV STEPS: Loop %d *****" % play_csv_loop)
            self.step_start_time = rospy.Time.now() # Performance monitor for each step 
                   
            # Process each row
            for row in self.csv_data:
            
                if interrupted:
                    rospy.loginfo("got exit request1, exiting.")
                    self.cleanup()
                    exit()
                    
                elapsed_time = rospy.Time.now() - self.step_start_time
                elapsed_sec = elapsed_time.to_sec()
                print("DBG: LAST STEP TOTAL TIME = %04.3f" % elapsed_sec )
                print("")
                self.step_start_time = rospy.Time.now()
                
                # Read a line from the CSV file
                self.script_row += 1
                servo_goal_required_radians = 0.0
                servo_goal_names = []
                servo_goal_values = []

                field_type = row['type']
                step = row['step']
                time = row['time']
                param1_str = row['param1'] 

                if field_type == 'sound': # TODO, get name from Param1
                    #sound_name = param1_str
                    rospy.loginfo("PLAYBACK: =========> Playing Sound")
                    #rospy.loginfo("PLAYBACK: =========> Playing Sound [%s]", sound_name)
                    # start playing wave file
                    pygame.mixer.music.load(self.music_file)
                    pygame.mixer.music.play(0)
                    #rospy.loginfo("Playing Music" )
                    #rospy.sleep(3.0)

                elif field_type == 'marker': # User marker, just display for tuning the script
                    marker_number = param1_str
                    rospy.loginfo("PLAYBACK: =========> User Marker [%s]", marker_number)

                elif field_type == 'turn': # Rotate robot at supplied speed/direction
                    try:
                        rospy.loginfo("PLAYBACK: =========> Wheel Turn [%s]", param1_str)
                        value = float(param1_str)
                        self.wheelTurn(value)
                    except ValueError:
                        rospy.logwarn("PLAYBACK: NO PARAM1 for TURN!" )

                elif field_type == 'lights': # Head and body light setting
                    try:
                        value = float(param1_str)
                        self.pub_light_mode.publish(value)
                    except ValueError:
                        rospy.logwarn("PLAYBACK: NO PARAM1 for lights!" )

                elif field_type == 'eye_color': # Eye Color setting
                    try:
                        value = float(param1_str)
                        self.pub_eye_color.publish(value)
                    except ValueError:
                        rospy.logwarn("PLAYBACK: NO PARAM1 for eye_color!" )

                #elif field_type == 'eye_mode':   #TODO-Optional
                #    eye_color = param1_str   

                elif field_type == 'speak':       # Say something
                    text_to_speak = param1_str # param1 contains the phrase to say
                    rospy.logwarn("TODO: SPEAKING: %s", text_to_speak )
                    # See follow_behavior for example.  Need to handle delay waiting for speech server to start?


                # Servo speeds are a mix of:
                # servo_speed_base: Overall movement speed (eg, walk speed)
                # servo_speed_step: Multiplied by the base speed to calculate speed of servos for the following CSV steps
                # servo_speed_mul:  Individual multiplier for each servo (speed ratio of servos related to each other)
                
                elif field_type == 'servo_speed_base':    
                    # Set base speed for all servos (which use a multiplier of this) E.g., overall walking speed)
                    try:
                        value = float(param1_str)
                        self.servo_speed_base = value
                        
                        index = 0
                        # Set all servos to the proper values
                        for servo_name in all_servo_joints:
                            mul_speed = self.servo_speed_multiplier[index]
                            final_speed = mul_speed * self.servo_speed_base * self.servo_speed_step
                            # rospy.loginfo("PLAYBACK Servo Base Speed: Setting servo %s, Value = %02.4f", servo_name, final_speed )
                            SetSingleServoSpeed(final_speed, servo_name)
                            index = index + 1
                        
                        # self.SetAllServos(self.servo_speed_base)                   
                        rospy.loginfo("PLAYBACK:  Step: [%s], Time: [%s], servo_speed_base set to: [%2.2f]", step, time, self.servo_speed_base)
                    except ValueError:
                        rospy.logwarn("PLAYBACK: NO PARAM1 for servo_speed_base!" )


                elif field_type == 'servo_speed_step': 
                    # Set speed multiplier for all servos for the following steps 
                    try:
                        value = float(param1_str)
                        self.servo_speed_step = value
                        
                        index = 0
                        # Set all servos to the proper values
                        for servo_name in all_servo_joints:
                            mul_speed = self.servo_speed_multiplier[index]
                            final_speed = mul_speed * self.servo_speed_base * self.servo_speed_step 
                            # rospy.loginfo("PLAYBACK Servo Base Step: Setting servo %s, Value = %02.4f", servo_name, final_speed )
                            SetSingleServoSpeed(final_speed, servo_name)
                            index = index + 1
                        rospy.loginfo("PLAYBACK:  Step: [%s], Time: [%s], servo_speed_step set to: [%2.2f]", step, time, self.servo_speed_step)

                    except ValueError:
                        rospy.logwarn("PLAYBACK: NO PARAM1 for servo_speed_step!" )

                elif field_type == 'servo_speed_mul':       # Set speed multiplier for each servo (speed = base_speed * speed_mul)
                    rospy.loginfo("PLAYBACK: Field type: Speed" )
                    step = row['step']
                    time = row['time']
                    comment = row['comment']
                    rospy.loginfo("PLAYBACK:  Step: [%s], Time: [%s], Comment: [%s]", step, time, comment)

                    # change any servo speeds with valid (non-NAN) values
                    self.SetSServoSpeedMultiplier('right_antenna',          row['right_antenna'])
                    self.SetSServoSpeedMultiplier('left_antenna',           row['left_antenna'])
                    self.SetSServoSpeedMultiplier('head_pan',               row['head_pan'])
                    self.SetSServoSpeedMultiplier('head_tilt',              row['head_tilt'])
                    self.SetSServoSpeedMultiplier('head_sidetilt',          row['head_sidetilt'])
                    self.SetSServoSpeedMultiplier('neck_raise',             row['neck_raise'])

                    #self.SetSServoSpeedMultiplier('leg_hip_lean',           row['leg_hip_lean'])
                    
                    self.SetSServoSpeedMultiplier('right_leg_hip_rotate',   row['right_leg_hip_rotate'])
                    self.SetSServoSpeedMultiplier('right_leg_thigh_lift',   row['right_leg_thigh_lift'])
                    self.SetSServoSpeedMultiplier('right_leg_knee_bend',    row['right_leg_knee_bend'])
                    self.SetSServoSpeedMultiplier('right_leg_ankle_rotate', row['right_leg_ankle_rotate'])

                    self.SetSServoSpeedMultiplier('left_leg_hip_rotate',    row['left_leg_hip_rotate'])
                    self.SetSServoSpeedMultiplier('left_leg_thigh_lift',    row['left_leg_thigh_lift'])
                    self.SetSServoSpeedMultiplier('left_leg_knee_bend',     row['left_leg_knee_bend'])
                    self.SetSServoSpeedMultiplier('left_leg_ankle_rotate',  row['left_leg_ankle_rotate'])

                elif field_type == 'servo_p_gain':       # Set PID for each servo
                    rospy.loginfo("PLAYBACK: Field type: PID P Gain" )
                    step = row['step']
                    time = row['time']
                    comment = row['comment']
                    rospy.loginfo("PLAYBACK:  Step: [%s], Time: [%s], Comment: [%s]", step, time, comment)

                    # change any servo P Gain with valid (non-NAN) values
                    self.SetPIDParam('p', 'right_antenna',          row['right_antenna'])
                    self.SetPIDParam('p', 'left_antenna',           row['left_antenna'])
                    self.SetPIDParam('p', 'head_pan',               row['head_pan'])
                    self.SetPIDParam('p', 'head_tilt',              row['head_tilt'])
                    self.SetPIDParam('p', 'head_sidetilt',          row['head_sidetilt'])
                    self.SetPIDParam('p', 'neck_raise',             row['neck_raise'])

                    #self.SetPIDParam('p', 'leg_hip_lean',           row['leg_hip_lean'])

                    self.SetPIDParam('p', 'right_leg_hip_rotate',   row['right_leg_hip_rotate'])
                    self.SetPIDParam('p', 'right_leg_thigh_lift',   row['right_leg_thigh_lift'])
                    self.SetPIDParam('p', 'right_leg_knee_bend',    row['right_leg_knee_bend'])
                    self.SetPIDParam('p', 'right_leg_ankle_rotate', row['right_leg_ankle_rotate'])

                    self.SetPIDParam('p', 'left_leg_hip_rotate',    row['left_leg_hip_rotate'])
                    self.SetPIDParam('p', 'left_leg_thigh_lift',    row['left_leg_thigh_lift'])
                    self.SetPIDParam('p', 'left_leg_knee_bend',     row['left_leg_knee_bend'])
                    self.SetPIDParam('p', 'left_leg_ankle_rotate',  row['left_leg_ankle_rotate'])
                    rospy.loginfo("PLAYBACK: P Gains Set.")
                    elapsed_time = rospy.Time.now() - self.step_start_time
                    elapsed_sec = elapsed_time.to_sec()
                    rospy.loginfo("DBG: Step Elapsed Time = %04.2f", elapsed_sec )
                    

                elif field_type == 'servo_i_gain':       # Set PID for each servo
                    rospy.loginfo("PLAYBACK: Field type: PID I Gain" )
                    step = row['step']
                    time = row['time']
                    comment = row['comment']
                    rospy.loginfo("PLAYBACK:  Step: [%s], Time: [%s], Comment: [%s]", step, time, comment)

                    # change any servo I Gain with valid (non-NAN) values
                    self.SetPIDParam('i', 'right_antenna',          row['right_antenna'])
                    self.SetPIDParam('i', 'left_antenna',           row['left_antenna'])
                    self.SetPIDParam('i', 'head_pan',               row['head_pan'])
                    self.SetPIDParam('i', 'head_tilt',              row['head_tilt'])
                    self.SetPIDParam('i', 'head_sidetilt',          row['head_sidetilt'])
                    self.SetPIDParam('i', 'neck_raise',             row['neck_raise'])

                    #self.SetPIDParam('i', 'leg_hip_lean',           row['leg_hip_lean'])
                    
                    self.SetPIDParam('i', 'right_leg_hip_rotate',   row['right_leg_hip_rotate'])
                    self.SetPIDParam('i', 'right_leg_thigh_lift',   row['right_leg_thigh_lift'])
                    self.SetPIDParam('i', 'right_leg_knee_bend',    row['right_leg_knee_bend'])
                    self.SetPIDParam('i', 'right_leg_ankle_rotate', row['right_leg_ankle_rotate'])

                    self.SetPIDParam('i', 'left_leg_hip_rotate',    row['left_leg_hip_rotate'])
                    self.SetPIDParam('i', 'left_leg_thigh_lift',    row['left_leg_thigh_lift'])
                    self.SetPIDParam('i', 'left_leg_knee_bend',     row['left_leg_knee_bend'])
                    self.SetPIDParam('i', 'left_leg_ankle_rotate',  row['left_leg_ankle_rotate'])
                    rospy.loginfo("PLAYBACK: I Gains Set.")
                    elapsed_time = rospy.Time.now() - self.step_start_time
                    elapsed_sec = elapsed_time.to_sec()
                    rospy.loginfo("DBG: Step Elapsed Time = %04.2f", elapsed_sec )

                elif field_type == 'servo_d_gain':       # Set PID for each servo
                    rospy.loginfo("PLAYBACK: Field type: PID D Gain" )
                    step = row['step']
                    time = row['time']
                    comment = row['comment']
                    rospy.loginfo("PLAYBACK:  Step: [%s], Time: [%s], Comment: [%s]", step, time, comment)

                    # change any servo D Gain with valid (non-NAN) values
                    self.SetPIDParam('d', 'right_antenna',          row['right_antenna'])
                    self.SetPIDParam('d', 'left_antenna',           row['left_antenna'])
                    self.SetPIDParam('d', 'head_pan',               row['head_pan'])
                    self.SetPIDParam('d', 'head_tilt',              row['head_tilt'])
                    self.SetPIDParam('d', 'head_sidetilt',          row['head_sidetilt'])
                    self.SetPIDParam('d', 'neck_raise',             row['neck_raise'])

                    #self.SetPIDParam('d', 'leg_hip_lean',           row['leg_hip_lean'])
                    
                    self.SetPIDParam('d', 'right_leg_hip_rotate',   row['right_leg_hip_rotate'])
                    self.SetPIDParam('d', 'right_leg_thigh_lift',   row['right_leg_thigh_lift'])
                    self.SetPIDParam('d', 'right_leg_knee_bend',    row['right_leg_knee_bend'])
                    self.SetPIDParam('d', 'right_leg_ankle_rotate', row['right_leg_ankle_rotate'])

                    self.SetPIDParam('d', 'left_leg_hip_rotate',    row['left_leg_hip_rotate'])
                    self.SetPIDParam('d', 'left_leg_thigh_lift',    row['left_leg_thigh_lift'])
                    self.SetPIDParam('d', 'left_leg_knee_bend',     row['left_leg_knee_bend'])
                    self.SetPIDParam('d', 'left_leg_ankle_rotate',  row['left_leg_ankle_rotate'])
                    rospy.loginfo("PLAYBACK: D Gains Set.")
                    elapsed_time = rospy.Time.now() - self.step_start_time
                    elapsed_sec = elapsed_time.to_sec()
                    rospy.loginfo("DBG: Step Elapsed Time = %04.2f", elapsed_sec )

                elif field_type == 'move':
                    step = row['step']
                    time = row['time']
                    comment = row['comment']

                    rospy.loginfo("PLAYBACK:  Step: [%s], Time: [%s], Comment: [%s]", step, time, comment)

                    # move any servos with valid (non-NAN) values
                    self.MoveServo(servo_goal_names, servo_goal_values, 'right_antenna',          pub_right_antenna,           row['right_antenna'])
                    self.MoveServo(servo_goal_names, servo_goal_values, 'left_antenna',           pub_left_antenna,            row['left_antenna'])
                    self.MoveServo(servo_goal_names, servo_goal_values, 'head_pan',               pub_head_pan,                row['head_pan'])
                    self.MoveServo(servo_goal_names, servo_goal_values, 'head_tilt',              pub_head_tilt,               row['head_tilt'])
                    self.MoveServo(servo_goal_names, servo_goal_values, 'head_sidetilt',          pub_head_sidetilt,           row['head_sidetilt'])
                    self.MoveServo(servo_goal_names, servo_goal_values, 'neck_raise',             pub_neck_raise,              row['neck_raise'])

                    #self.MoveServo(servo_goal_names, servo_goal_values, 'leg_hip_lean',           pub_leg_hip_lean,            row['leg_hip_lean'])

                    self.MoveServo(servo_goal_names, servo_goal_values, 'right_leg_hip_rotate',   pub_right_leg_hip_rotate,    row['right_leg_hip_rotate'])
                    self.MoveServo(servo_goal_names, servo_goal_values, 'right_leg_thigh_lift',   pub_right_leg_thigh_lift,    row['right_leg_thigh_lift'])
                    self.MoveServo(servo_goal_names, servo_goal_values, 'right_leg_knee_bend',    pub_right_leg_knee_bend,     row['right_leg_knee_bend'])
                    self.MoveServo(servo_goal_names, servo_goal_values, 'right_leg_ankle_rotate', pub_right_leg_ankle_rotate,  row['right_leg_ankle_rotate'])

                    self.MoveServo(servo_goal_names, servo_goal_values, 'left_leg_hip_rotate',    pub_left_leg_hip_rotate,     row['left_leg_hip_rotate'])
                    self.MoveServo(servo_goal_names, servo_goal_values, 'left_leg_thigh_lift',    pub_left_leg_thigh_lift,     row['left_leg_thigh_lift'])
                    self.MoveServo(servo_goal_names, servo_goal_values, 'left_leg_knee_bend',     pub_left_leg_knee_bend,      row['left_leg_knee_bend'])
                    self.MoveServo(servo_goal_names, servo_goal_values, 'left_leg_ankle_rotate',  pub_left_leg_ankle_rotate,   row['left_leg_ankle_rotate'])

                    # Param1 holds accuracy of servo goal needed to go to next step
                    try:
                        servo_goal_required_radians = float(param1_str)
                    except ValueError:
                        servo_goal_required_radians = 0.0
                        rospy.loginfo("DBG: Servo Goal Accuracy not set in Param1. Not tracking servo position.")
                    if servo_goal_required_radians != 0.0:
                        rospy.loginfo("Servo Goal Accuracy set to %2.3f radians. Tracking Servo Positions!", servo_goal_required_radians )

                elif field_type == '':
                    rospy.loginfo("skipping blank line (no field type)")
                elif field_type == '#':
                    rospy.loginfo("skipping comment line (field type #)")
                else:
                    rospy.logwarn("ERROR! Unknown field_type [%s]", field_type)


                # Done with this row. Handle script playback timing
                next_step_interval = 0.0
                try:
                    next_step_interval = float(row['time'])
                except ValueError:
                    pass # Blanks = zero

                script_score_time += next_step_interval
                elapsed_time = rospy.Time.now() - self.step_start_time
                elapsed_sec = elapsed_time.to_sec()
                rospy.loginfo("DBG: Step Elapsed Time = %04.2f", elapsed_sec )
                rospy.loginfo("DBG: Waiting for next step...") 
                
                while True: # wait until one of the following allows next step to proceed
                
                    if interrupted:
                        rospy.loginfo("got exit request4, exiting.")
                        self.cleanup()
                        exit()

                    elif next_step_interval == 0:  
                        # immediately move to next step. Don't wait for single step mode
                        rospy.loginfo("DBG: interval is zero, going to next step immediately")
                        break
                    
                    elif self.single_step_mode:
                        if self.next_step:
                            self.next_step = False
                            break
                        else:
                            #rospy.loginfo("...wait for trigger...")
                            rospy.sleep(0.2)
                         
                    else:
                        # wait for servos to reach positions or time to expire, whichever comes first
                        elapsed_time = rospy.Time.now() - self.script_start_time
                        elapsed_sec = elapsed_time.to_sec()
                        remaining_delay = script_score_time - elapsed_sec
                                                
                        # if moving servos, see if they have reached their goals
                        if servo_goal_required_radians != 0.0:
                            # print("-------------------------------------------")
                            # print("DBG: servo_goal_required_radians = ", servo_goal_required_radians)
                            if servo_goal_names: # check if list has something in it
                                # print("DBG: servo_goal_names = ", servo_goal_names)
                                # print("DBG: servo_goal_values = ", servo_goal_values)
                            
                                # only current_position is used, because 'goal_position' isn't updated until after the servo receives the new goal! 
                                (current_position, velocity, effort, goal_position, error) = self.call_return_joint_states(servo_goal_names)

                                goal_index = 0
                                biggest_delta = 0.0
                                slowest_servo_name = ''

                                # Alternative approach: for servo_name, current, goal in zip(servo_goal_names, current_position, servo_goal_values):
                                for servo_name in servo_goal_names:
                                    goal_value = servo_goal_values[goal_index]
                                    current_value = current_position[goal_index]
                                    delta = math.fabs(goal_value - current_value)
                                    # rospy.loginfo("DBG: %s Goal Value = % 2.3f, Current Value = % 2.3f, Delta = % 2.3f", servo_name, goal_value, current_value, delta )
                                    if delta > biggest_delta:
                                        biggest_delta = delta
                                        slowest_servo_name = servo_name
                                    goal_index = goal_index + 1

                                if biggest_delta < servo_goal_required_radians:
                                    rospy.loginfo("DBG: Required Delta = % 2.3f (% 2.3f), Current Largest Delta = % 2.3f (% 2.3f)", 
                                        servo_goal_required_radians, (servo_goal_required_radians * RAD_DEG), biggest_delta, (biggest_delta * RAD_DEG) )
                                    rospy.loginfo("DBG: Servo move goals reached!")
                                    
                                    elapsed_time = rospy.Time.now() - self.step_start_time
                                    elapsed_sec = elapsed_time.to_sec()
                                    rospy.loginfo("DBG: Servo move total elapsed Time = % 4.2f", elapsed_sec )
                                    break
                                else:
                                    rospy.loginfo("DBG: Required Delta = % 2.3f (% 2.3f), Current Largest Delta = % 2.3f (% 2.3f)  Slowest Servo = %s",
                                        servo_goal_required_radians, (servo_goal_required_radians * RAD_DEG), 
                                        biggest_delta, (biggest_delta * RAD_DEG), slowest_servo_name )

                            if remaining_delay < 0.05: # tune this for latency
                                print("")
                                rospy.loginfo("****************** SERVO MOVE TIMED OUT! *********************")
                                print("")
                                break
                            else:
                                rospy.sleep(0.01)

                        else: # servo movement not being tracked
                            # check elapsed time, sleep as needed
                            if remaining_delay > 1.0:
                                #rospy.loginfo("...sleep big...")
                                rospy.sleep(0.4)
                            elif remaining_delay > 0.05: # tune this for latency
                                #rospy.loginfo("...sleep small...")
                                rospy.sleep(0.025)
                            else:
                                rospy.loginfo("DBG: Step Timeout reached!")
                                break

                # end of while

                #elapsed_time = rospy.Time.now() - self.script_start_time 
                #elapsed_sec = elapsed_time.to_sec() # script cumulative total time
                #stars = "*"
                #for i in range(int((elapsed_sec * 10))):
                #    stars = stars + '*'
                #print(("Script Time: %04.2f   " + stars) % elapsed_sec)


        self.cleanup()


# Radian Reference
#  0 deg         0.0
#  1 deg         0.0174533
# 10 deg         0.174533
# 15 deg         0.261799
# 20 deg         0.349066
        
# ------------------------------------------------------------------
if __name__=='__main__':

    # capture SIGINT signal, e.g., Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    csv_filename = None
    csv_loops = 0
    total = len(sys.argv)
    #cmdargs = str(sys.argv)
    if total > 1:
        csv_filename = sys.argv[1].lower() # Optional filename
        # print("DBG Got filename:", csv_filename)
    if total > 2:
        arg2 = sys.argv[2] # Optional loop repeats. if -1, single step mode 
        csv_loops = int(arg2)
        # print("DBG Got CSV Loops: %d" % csv_loops)
 
    try:
        playback_servos = PlaybackServoPositions(csv_filename, csv_loops)
        playback_servos.run()

    except rospy.ROSInterruptException:
        pass

    #rospy.spin()


