#!/usr/bin/env python3
# Walk Engine - Generates movements for walking
#
# TODO LIST:
# Set servo speeds: Knee Bend = 3.0, others 1.5
 
# TODO: set_servo_pid.py left_wheel_motor_joint 64 0 0


import rospy
import logging
import time
import math
import sys
import signal
import numpy as np


from sensor_msgs.msg import Joy
#from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import JointState # single servo messages
from dynamixel_msgs.msg import JointStateArray

from std_msgs.msg import Float32
from std_msgs.msg import UInt16
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from geometry_msgs.msg import Point32

# EB Robot ONLY
#from dynamixel_controllers.srv import TorqueEnable, SetServoTorqueLimit, SetSpeed
from eb_servos.set_pose import *    # for RobotPose
from eb_servos.servo_joint_list import all_servo_joints, all_non_wheel_joints, head_joints, right_leg_joints, left_leg_joints
from eb_servos.head_servo_publishers import *
from eb_servos.leg_servo_publishers import *

#from eb_servos.standard_servo_positions import *
from eb_servos.set_servo_speed import *
from eb_servos.set_servo_torque import *
from eb_servos.usb_latency_test import *
from eb_servos.srv import ReturnJointStates

g_interrupted = False
RIGHT_LEG = 2
LEFT_LEG  = 1

MODE_POSE0 = 0
MODE_POSE1 = 1
MODE_POSE2 = 2
MODE_POSE3 = 3
MODE_POSE4 = 4
MODE_STOP  = 5
MODE_WALK  = 6

DEBUG_HEAD_DOWN = False
WHEEL_MOTOR_MODE = True
BAL_PID_MODE = False
BAL_DOUBLE_WHEEL = True

DYNAMIXEL_64_MAX_SPEED = 6.762 # radians per second

BAL_PID_DEFAULT_KP  = 0.0 # 10.0
BAL_PID_DEFAULT_KI  = 0.0 # 40
BAL_PID_DEFAULT_KD  = 0.0 # 0.05
BAL_DEFAULT_IMU_OFFSET = 2.2 # 5.8 #10.4 # 3.0
#BAL_ANKLE_NEUTRAL_POSITION = -0.700 # Position servo for robot not moving TODO fix this for all poses!
BAL_ANKLE_NEUTRAL_OFFSET = -0.0 # 800 # biger = lean back

# sampleTime  0.005
# targetAngle -2.5




def signal_handler(signal, frame):
    global g_interrupted
    g_interrupted = True


class WalkEngine():
    def __init__(self):
        rospy.init_node('walk_engine')
        print("*************** Starting Walk Engine **************")
        if usb_latency_test() == False:
            sys.exit()

        # CONSTANTS
        # Total length hip to ankle is thigh_length + shin length, which happen to be the same
        self.thigh_length = 144.38  # Actual bone length of thigh and also shin
        self.min_leg_height = 180   # mm measured, 10 mm from touching body
        self.max_leg_height = 270   # mm at Pose4. 280 mm = Pose5   
        self.foot_lift_distance = 30 # mm
        # self.leg_down_default_height = 200 + self.foot_lift_distance

        self.imu_roll_hardware_offset_degrees = 0.0 # -0.2 Tune for even walking (on graph)
        self.imu_pitch_hardware_offset_degrees = BAL_DEFAULT_IMU_OFFSET
         # 
        self.x_turn_offset = 0.2 # Tune right/left x to make robot walk straight
        self.x_offset = 2.0 # -1.4 # Tune x to make robot stay in one place until told to walk
        self.x_start_cycle = 4 # start walking after N cycles
        
        # Variables        

        self.joint_state = JointStateArray()
        self.robotpose = RobotPose("eb_walk")
        self.record_mode = 'all' # 'changed' = servos that moved > threshold
        self.step_number = 0
        self.marker_index = 0

        self.button_pressed = 0xFF # no button pressed
        self.saved_servo_position = 0.0
        self.servo_torque_enabled_right = True
        self.servo_torque_enabled_left = True
        self.servo_torque_enabled_head = True
        self.servo_torque_enabled_current = True
        self.servo_number = 1  # default to first servo
        self.move_servo_pressed = False
        self.joy_amount_x = 0.0
        self.set_pose_topic = '/set_pose'
        self.right_joy_enable = False
        self.left_joy_enable = False
        self.timer_delay = 0.0
        self.walking_servo_speed = 2.0
        self.imu_pitch_offset = 0.0
        self.imu_pitch_compensated = 0.0
        
        # NEW STUFF
        self.robot_pose = 0
        self.walk_mode = 0  # Idle
        self.walking_enabled = False
        self.enable_balancing = False
        self.right_forward_joystick = 0.0
        self.right_turn_joystick = 0.0
        self.left_height_joystick = 0.0
        self.default_step_height = 11.5 # 8.0
        self.first_step_height = 12.0
        self.height_increase = 0.0 # for tuning
        
        self.imu_roll = 0.0     # positive = roll left           
        self.imu_pitch = 0.0    # positive = pitch forward           
        self.imu_yaw = 0.0      # positive = yaw left
        self.imu_cycle = 1
        self.imu_pos_peak = 0.0
        self.imu_neg_peak = 0.0
        self.last_imu_pos_peak = 0.0
        self.last_imu_neg_peak = 0.0
        self.body_tilt_compensation = 0.0 # compensate for tilt, using IMU feedback
        self.speed_kick = 0.0
        self.cycle_count = 0
        
        self.first_step = True
        self.step_height = self.default_step_height
        self.step_length = 0.0 # stride length in mm
        self.step_flag = 0
        self.left_step_flag = 0
        self.right_step_flag = 0
        self.timer1 = 0
        self.timer2 = 0
        self.timer3 = 0
        self.startMillis = 0
        self.pidMillis = 0
        self.currentMillis = 0
        self.previousStepMillis = 0
        self.previousRightStepMillis = 0
        self.previousLeftStepMillis = 0  
        self.left_z = 0
        self.left_x = 0      
        self.left_y = 0
        self.right_z = 0
        self.right_x = 0
        self.right_y = 0
        self.hip_turn = 0.0
        
        self.bal_kp = BAL_PID_DEFAULT_KP
        self.bal_ki = BAL_PID_DEFAULT_KI
        self.bal_kd = BAL_PID_DEFAULT_KD
        self.bal_target_angle = 0.0
        self.bal_current_angle = 0.0
        self.bal_prev_angle = 0.0
        self.bal_error_sum = 0.0
        self.bal_motor_command = 0.0
        self.bal_joy_speed_cmd = 0.0
        self.bal_joy_turn_cmd = 0.0
        self.bal_speed_ramp = 0.0
        
        self.dbg_flipflop = True
        self.robot_fall_detected = False
        self.right_ankle_effort = 0.0
        self.left_ankle_effort = 0.0
        self.braking_overshoot = 0.0
        self.last_speed_cmd = 0.0
        
        
        self.slider01 = 0.0
        self.slider02 = 0.0
        self.slider03 = 0.0
        self.slider04 = 0.0
        self.slider05 = 0.0
        self.slider06 = 0.0
        self.slider07 = 0.0
        self.slider08 = 0.0
        self.slider09 = 0.0
        self.slider10 = 0.0
        self.slider11 = 0.0
        self.slider12 = 0.0

                      
        self.half_control_range = ((self.max_leg_height - self.min_leg_height) / 2.0)    # up to +/- this value
        self.mid_height = self.min_leg_height + ((self.max_leg_height - self.min_leg_height) / 2.0)
        # self.max_extension_from_min = self.max_leg_height - (self.min_leg_height + self.foot_lift_distance)
        self.enable_body_tilt_compensation = False
        
        self.loop_time = rospy.get_rostime() # debug time for full walk cycle
        self.loop_time_result = 0

        # SUBSCRIBERS
        
        # Get Walk commands (TODO - FUTURE)
        # walk_command_sub = rospy.Subscriber("/walk", UInt16, self.walk_command_callback)


        # TODO - for now, use joystick to tune walking
        joystick_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)

        # get imu updates from Arduino BNO086
        imu_sub = rospy.Subscriber("/imu_orientation", Point32, self.imu_callback)

        # Subscribe to servo position messages instead of service becuase we need to know the instant a new update is available
        servo_sub = rospy.Subscriber('/joint_states_array', JointStateArray, self.joint_state_cb)

        # slider0 Bars for tuning walking parameters
        slider01_sub  = rospy.Subscriber('/value1',  Float32, self.slider01_cb)
        slider02_sub  = rospy.Subscriber('/value2',  Float32, self.slider02_cb)
        slider03_sub  = rospy.Subscriber('/value3',  Float32, self.slider03_cb)
        slider04_sub  = rospy.Subscriber('/value4',  Float32, self.slider04_cb)
        slider05_sub  = rospy.Subscriber('/value5',  Float32, self.slider05_cb)
        slider06_sub  = rospy.Subscriber('/value6',  Float32, self.slider06_cb)
        slider07_sub  = rospy.Subscriber('/value7',  Float32, self.slider07_cb)
        slider08_sub  = rospy.Subscriber('/value8',  Float32, self.slider08_cb)
        slider09_sub  = rospy.Subscriber('/value9',  Float32, self.slider09_cb)
        slider10_sub = rospy.Subscriber('/value10',  Float32, self.slider10_cb)
        slider11_sub = rospy.Subscriber('/value11',  Float32, self.slider11_cb)
        slider12_sub = rospy.Subscriber('/value12',  Float32, self.slider12_cb)
        # handle mode/pose button selections on the GUI
        mode_sub     = rospy.Subscriber('/walk_mode', Int32, self.walk_mode_cb)


 
        # PUBLISHERS
        
        # Note that servo publishers are defined in includes
        self.pub_imu_roll =         rospy.Publisher('/imu_left', Float64, queue_size=1)
        self.pub_imu_pitch =        rospy.Publisher('/imu_pitch', Float64, queue_size=1)
        self.pub_imu_yaw =          rospy.Publisher('/imu_yaw', Float64, queue_size=1)
        # self.pub_compensated_imu =  rospy.Publisher('/imu_comp', Float64, queue_size=1)
        # self.pub_body_tilt =        rospy.Publisher('/body_tilt', Float64, queue_size=1)
        self.pub_step_cycle =       rospy.Publisher('/step_cycle', Float64, queue_size=1)
        self.pub_timer2 =           rospy.Publisher('/timer2', Float64, queue_size=1)
        self.pub_timer3 =           rospy.Publisher('/timer3', Float64, queue_size=1)

        self.pub_right_cycle =      rospy.Publisher('/right_cycle', Float64, queue_size=1)
        self.pub_right_up =         rospy.Publisher('/right_up', Float64, queue_size=1)
        self.pub_right_fwd =        rospy.Publisher('/right_fwd', Float64, queue_size=1)
        self.pub_right_knee =       rospy.Publisher('/right_knee', Float64, queue_size=1)
        #self.pub_right_ankle =      rospy.Publisher('/right_ankle', Float64, queue_size=1)
        self.pub_right_thigh =      rospy.Publisher('/right_thigh', Float64, queue_size=1)
        self.pub_right_servo_cmd =  rospy.Publisher('/right_servo_cmd', Float64, queue_size=1)

        self.pub_right_knee_cmd =   rospy.Publisher('/right_knee_cmd', Float64, queue_size=1)
        #self.pub_right_ankle_cmd =  rospy.Publisher('/right_ankle_cmd', Float64, queue_size=1)
        self.pub_right_thigh_cmd =  rospy.Publisher('/right_thigh_cmd', Float64, queue_size=1)

        
        self.pub_left_cycle =       rospy.Publisher('/left_cycle', Float64, queue_size=1)
        self.pub_left_up =          rospy.Publisher('/left_up', Float64, queue_size=1)
        self.pub_left_fwd =         rospy.Publisher('/left_fwd', Float64, queue_size=1)
        self.pub_left_knee =        rospy.Publisher('/left_knee', Float64, queue_size=1)
        #self.pub_left_ankle =       rospy.Publisher('/left_ankle', Float64, queue_size=1)
        self.pub_left_thigh =       rospy.Publisher('/left_thigh', Float64, queue_size=1)
        self.pub_left_servo_cmd =   rospy.Publisher('/left_servo_cmd', Float64, queue_size=1)

        self.pub_left_knee_cmd =    rospy.Publisher('/left_knee_cmd', Float64, queue_size=1)
        #self.pub_left_ankle_cmd =   rospy.Publisher('/left_ankle_cmd', Float64, queue_size=1)
        self.pub_left_thigh_cmd =   rospy.Publisher('/left_thigh_cmd', Float64, queue_size=1)
        
        self.pub_pid_pcalc =        rospy.Publisher('/pid_pcalc', Float64, queue_size=1)
        self.pub_pid_icalc =        rospy.Publisher('/pid_icalc', Float64, queue_size=1)
        self.pub_pid_dcalc =        rospy.Publisher('/pid_dcalc', Float64, queue_size=1)
        self.pub_pid_error =        rospy.Publisher('/pid_error', Float64, queue_size=1)
        self.pub_pid_error_sum =    rospy.Publisher('/pid_error_sum', Float64, queue_size=1)
        self.pub_motor_command =    rospy.Publisher('/pid_motor_cmd', Float64, queue_size=1)

        # Dynamixel motor control:
        self.pub_wheel_cmd_right = rospy.Publisher('/right_wheel_motor_joint/command', Float64, queue_size=1)
        self.pub_wheel_cmd_left  = rospy.Publisher('/left_wheel_motor_joint/command',  Float64, queue_size=1)
        
        # Arduino motor control:
        #self.pub_wheel_cmd_right = rospy.Publisher('/wheel_speed_right', Int16, queue_size=1)
        #self.pub_wheel_cmd_left = rospy.Publisher('/wheel_speed_left', Int16, queue_size=1)
        

        self.start_time = rospy.get_rostime()
        
        SetServoTorque(1.0, all_servo_joints) # 1.0 = max torque
        self.set_servo_speed(self.walking_servo_speed) # speed = 2.0 # 1.5?
        
        self.robot_pose = self.robotpose.get_initial_pose()
        self.bal_ankle_neutral_position = self.robotpose.get_ankle_neutral_position(self.robot_pose) + BAL_ANKLE_NEUTRAL_OFFSET
        
        self.pub_wheel_cmd_right.publish(0)
        self.pub_wheel_cmd_left.publish(0)

        
        print("")
        print("JoyStick Buttons:")
        print("   Green:   Enable Movement")
        print("   Red:     Stop Movement")
        print("   Blue:    Test Mode")
        print("   Yellow:  Walk Mode")
        print("")
        print("   Top Right Trigger: Adjust step height by Joystick")
        print("   Top Left Trigger:  Adjust step length by Joystick")
        print("")         
        
        print("waiting for return_joint_states service...")
        rospy.wait_for_service("return_joint_states")
        print("service found.")
        print("Walk Engine ready.")       

        
        #------------------------------------ Init end --------------------------------------        
        

    def __del__(self):
        print("__del__ called on exit.")

    def clamp(self, value, max_value): # clamp between pos and neg max_value
        return max(min(value, max_value), -max_value)


    def set_servo_speed(self, speed):
        #print("********** SETTING SERVO SPEED AND TORQUE ******************")
        #speed = 1.5 # 0.8 # 1.5?
        SetServoSpeed( speed, all_non_wheel_joints)  # TODO - This will override whatever camera tracker is doing?
        SetSingleServoSpeed((speed * 2.0), 'right_leg_knee_bend_joint') # Knee has twice distance to travel
        SetSingleServoSpeed((speed * 2.0), 'left_leg_knee_bend_joint')
        #print("******** DONE SETTING SERVO SPEED AND TORQUE ****************")
    
    def walk_height_callback(self, msg):
        self.walk_height = msg.data
        # move legs into position
        

    def walk_command_callback(self, msg):
        rospy.loginfo("WALK: got walk command from user")

        # print("DBG: Message.data = ", msg.data)
        
        self.walk_mode = msg.data


    def set_pose_cb(self, msg):
        rospy.loginfo("got new pose from user")

        # print("DBG: Message.data = ", msg.data)
        #rx_data = msg.data
        
        #print("rx = ", rx_data)
        
        pose = msg.data
        self.set_pose(pose)
        

    def set_pose(self, pose):
        rospy.loginfo("----------> New Pose set to: %d" % pose) # 0 = sleep position, 1 = Sitting with head up, etc.
        slowest_servo_speed = 0.3
        self.robotpose.move(pose, slowest_servo_speed)
        # update internal status
        self.robot_pose = pose
        self.bal_ankle_neutral_position = self.robotpose.get_ankle_neutral_position(pose) + BAL_ANKLE_NEUTRAL_OFFSET
    
        
    # ------------------------------------------------------------------------------
    # UTILITIES

    def constrain(self, in_val, min_val, max_val):
        if min_val > max_val:
            print("ERROR - Constrain: min_val > max_val!")
            return in_val
        max_constrained = min(in_val, max_val)
        constrained_val = max(max_constrained, min_val)
        return constrained_val      

    def deadband(self, in_val, band_val):
        if (in_val > (band_val * -1)) and (in_val < band_val):
            #print("DEADBAND: constraining value!")
            return 0.0
        else:
            return in_val
            
    def update_ankle_servos(self, speed_cmd):
        # apply ankle servo changes for moving robot             
        #if self.enable_balancing and not self.robot_fall_detected:
        pub_right_leg_ankle_rotate.publish(self.bal_ankle_neutral_position + (speed_cmd / 10.0))
        pub_left_leg_ankle_rotate.publish( self.bal_ankle_neutral_position + (speed_cmd / 10.0))
    

    def wheel_motor_ramp(self):
        # called each time through the fast loop. Speed command is -1.0 to +1.0
        ramp_speed_seconds = 2.0 # time for full ramp from stop to max speed
        ramp_rate = 0.02 / ramp_speed_seconds # 0.02 How much to step each time through the 20ms loop (50 times / second)
        speed_cmd =  self.bal_joy_speed_cmd      


        # TODO - lean servos when accelerating?            
        #self.last_speed_cmd = self.bal_joy_speed_cmd
        
        speed_delta = speed_cmd - self.bal_speed_ramp
        if False: #speed_delta !=0:
            pitch_radians = speed_delta / 10.0
            pub_right_leg_ankle_rotate.publish(self.bal_ankle_neutral_position + pitch_radians)
            pub_left_leg_ankle_rotate.publish( self.bal_ankle_neutral_position + pitch_radians)


        # Ramp the motor speeds        
        if self.bal_speed_ramp < speed_cmd:
            # Speeding up forward or slowing in reverse
            self.bal_speed_ramp = min((self.bal_speed_ramp + ramp_rate), speed_cmd) 
            #self.update_ankle_servos(self.bal_speed_ramp)
            
        elif self.bal_speed_ramp > speed_cmd:
            self.bal_speed_ramp = max((self.bal_speed_ramp - ramp_rate), speed_cmd) 
            #self.update_ankle_servos(self.bal_speed_ramp)

        if self.bal_speed_ramp != 0.0:
            print("RAMP: joy=% 2.2f  ramp=% 2.2f  imu=% 2.2f" % 
            (self.bal_joy_speed_cmd, self.bal_speed_ramp, self.imu_pitch_compensated))

            
        # Return the ramped motor command
        return self.bal_speed_ramp 
        
        
    def bal_motor_ramp(self):
        # called each time through the fast loop. Speed command is -1.0 to +1.0
        # Phase 1: ramp up the ankle servos
        ramp_speed_seconds = 0.5 # time for full ramp from stop to max speed
        ramp_rate = 0.02 / ramp_speed_seconds # How much to step each time through the 20ms loop (50 times / second)
        motor_adder = 0.0
        
        # monitor when braking needed
        if self.bal_joy_speed_cmd != 0.0:
            self.braking_overshoot = 0.0  # only brake when stopping
        elif self.last_speed_cmd > 0.0:  
            # forward stop command received
            self.braking_overshoot = self.slider02/100.0 * -1.0  # 0.2 # TODO Tune this value
        elif self.last_speed_cmd < 0.0:  
            # backward stop command received
            self.braking_overshoot = self.slider02/100.0  # 0.2 # TODO Tune this value
            
        self.last_speed_cmd = self.bal_joy_speed_cmd

        if abs(self.bal_motor_command) < 1.5:
            # motor close to stopped. Turn off brake
            self.braking_overshoot = 0.0  
            
        speed_cmd =  self.bal_joy_speed_cmd + self.braking_overshoot      

        # now handle the ramp        
        if self.bal_speed_ramp < speed_cmd:
            # Speeding up forward or slowing in reverse
            self.bal_speed_ramp = min((self.bal_speed_ramp + ramp_rate), speed_cmd) 
            self.update_ankle_servos(self.bal_speed_ramp)
            
        elif self.bal_speed_ramp > speed_cmd:
            self.bal_speed_ramp = max((self.bal_speed_ramp - ramp_rate), speed_cmd) 
            self.update_ankle_servos(self.bal_speed_ramp)

        #else:
            #pass
        if True:
            print("RAMP: cmd=% 2.2f  ramp=% 2.2f  imu=% 2.2f  brake=% 2.2f  motor=% 2.2f" % 
            (self.bal_joy_speed_cmd, self.bal_speed_ramp, self.imu_pitch_compensated, self.braking_overshoot, self.bal_motor_command))
            
        # Phase 2?: ramp down the ankle servos back to neutral, while ramping up direct motor speed                    
        return motor_adder    



    def servo_name(self):
        return all_non_wheel_joints[self.servo_number-first_servo_number] # list is zero offset

    def call_return_joint_states(self, joint_names):
    
        rospy.wait_for_service("return_joint_states")
        try:
            s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
            resp = s(joint_names)
        except rospy.ServiceException as e:
            rospy.logwarn("%s: Error when calling return_joint_states: %s" % ("eb_walk", e))

        #print("************* RESULTS *****************")


        for (ind, joint_name) in enumerate(joint_names):
            if(not resp.found[ind]):
                rospy.logwarn("%s: joint %s not found!" % ("eb_walk", joint_name ))
            #else:
            #    print("Joint found: %s, Position = %2.3f" % (joint_name, resp.position[ind]))

        return (resp.position, resp.velocity, resp.effort, resp.goal_pos, resp.error)

    def get_servo_position(self, servo_name):
        # Single servo name, but service expects an array of names
        servo_names = [servo_name]
        # print(servo_names)        
        (position, velocity, effort, goal_pos, error) = self.call_return_joint_states(servo_names)
        servo_pos = position[0]
        return servo_pos

    def get_servo_full_status(self, servo_name):
        # Single servo name, but service expects an array of names
        servo_names = [servo_name]
        # print(servo_names)        
        (position, velocity, effort, goal_pos, error) = self.call_return_joint_states(servo_names)
        return (position[0], velocity[0], effort[0], goal_pos[0], error[0])

    
    def imu_callback(self, data):
        #print("got imu: ", data) 
        self.imu_roll = data.x + self.imu_roll_hardware_offset_degrees + (self.slider07/10.0)
        self.imu_pitch = data.y + self.imu_pitch_hardware_offset_degrees + (self.slider11/100.0)
        self.imu_pitch_compensated = self.imu_pitch
        if False: # abs(self.imu_pitch) < 2.0:
            self.imu_pitch_compensated = self.imu_pitch - self.imu_pitch_offset # auto-tuned IMU
        self.imu_yaw = data.z
        

    def joint_state_cb(self, data):
        # Gets all servo updates. Filter for the body tilt
        # Note: this is faster than monitoring single servo, or calling the service
        
           
        knee_center = 1.37
        #ankle_center = 0.66
        thigh_center = 0.68
            
        for i in range(len(data.name)):
            # print( "DBG servo_update_callback: joint =  [" + data.name[i] + "]")


            # get effort of ankle servos    
            if data.name[i] == 'left_leg_ankle_rotate_joint':
                servo_pos = data.position[i] * -1.0
                self.left_ankle_effort = data.effort[i] # * -1.0
                #print("DBG: left ankle reported pos = %02.2f, effort = %02.2f" % (servo_pos, self.left_ankle_effort))
                #self.pub_left_ankle.publish(((servo_pos - thigh_center) * 1000) + 600)
                
            elif data.name[i] == 'right_leg_ankle_rotate_joint':
                self.right_ankle_effort = data.effort[i] # * -1.0
            
            # get positions of some servos and publish them for debug:
            #if not self.walking_enabled: 
            #    return # pause Graph
        
            elif data.name[i] == 'left_leg_knee_bend_joint':
                servo_pos = data.position[i]
                #print("DBG: left knee reported = %02.2f" % servo_pos)
                #TODO this is an ERROR????  self.pub_left_knee.publish(((servo_pos - knee_center) * 500) + 600)
                # self.pub_left_knee.publish((servo_pos * 1000) - 1000)
    
            elif data.name[i] == 'left_leg_thigh_lift_joint':
                servo_pos = data.position[i] * -1.0
                #print("DBG: left thigh reported = %02.2f" % servo_pos)
                #TODO this is an ERROR???? self.pub_left_thigh.publish(((servo_pos - thigh_center) * 1000) + 600)


            elif data.name[i] == 'right_leg_knee_bend_joint':
                servo_pos = data.position[i]
                #print("DBG: right knee reported = %02.2f" % servo_pos)
                #TODO this is an ERROR???? self.pub_right_knee.publish(((servo_pos - knee_center) * 500) - 400)
                # self.pub_right_knee.publish((servo_pos * 1000) - 1000)
    
            elif data.name[i] == 'right_leg_thigh_lift_joint':
                servo_pos = data.position[i] * -1.0
                #print("DBG: right thigh reported = %02.2f" % servo_pos)
                #TODO this is an ERROR???? self.pub_right_thigh.publish(((servo_pos - thigh_center) * 1000) - 400)
                



        #print("")

    
           
    # =============================== JOYSTICK CALLBACK =================================
    def joy_callback(self, data):
        # Joystick messages
        # Only one button supported at a time
        # print("")
        # print("--------------------------")
        # self.button_pressed = -1
        #for i in range(0,12): 
        #    print(i)               
        #    if data.buttons[i] == 1:  # Button Pressed
        #        self.button_pressed = i
        #        # break

        debug_joy_buttons = False
        if debug_joy_buttons:
            if data.buttons[0] == 1:
                print("X Button - Blue")
            if data.buttons[1] == 1:
                print("A Button - Green")
            if data.buttons[2] == 1:
                print("B Button - Red")
            if data.buttons[3] == 1:
                print("Y Button - Yellow")
            if data.buttons[4] == 1:
                print("Left Top trigger")
            if data.buttons[5] == 1:
                print("Right Top trigger")
            if data.buttons[6] == 1:
                print("Left Bottom Trigger")
            if data.buttons[7] == 1:
                print("Right Bottom trigger")
            if data.buttons[8] == 1:
                print("Back Button")
            if data.buttons[9] == 1:
                print("Start Button")
            if data.buttons[10] == 1:
                print("Button 10?")
            if data.buttons[11] == 1:
                print("Button 11?")

        # handle joystick input

        if data.buttons[0] == 1:         # X - Blue:   Stand in place (Test Mode)
            self.set_walk_mode(MODE_STOP)
        elif data.buttons[3] == 1:       # Y - Yellow: Start walking
            self.set_walk_mode(MODE_WALK)
        elif data.buttons[2] == 1:         # B - Red:    TODO TODO Sit down (sleep position)
            self.set_walk_mode(MODE_STOP) # TODO MODE_POSE4
        elif data.buttons[1] == 1:       # A - Green:  Stand ready Pose
            self.set_walk_mode(MODE_POSE4)
         
            
        top_left_trigger = False
        top_right_trigger = False
        top_left_trigger = data.buttons[4] # Left Top trigger 
        top_right_trigger = data.buttons[5] # Right Top trigger 

        debug_joy_buttons = False
        if debug_joy_buttons:
            print("Joy button command summary:")
            print("  walking enabled =     ", self.walking_enabled)
            print("  top_left_trigger =  ", top_left_trigger)
            print("  top_right_trigger = ", top_right_trigger)

        # Joystick inputs (+1.0 to -1.0) Up/Left is positive
        debug_joy_axis = False
        if debug_joy_axis:
            # WARNING: If joystick does not match this, make sure the "Mode" light is OFF!
            #print("Raw Data Axis", data.axes)
            print("Joy0 - Left  horz:  %2.3f" % (data.axes[0]))
            print("Joy1 - Left  vert   %2.3f" % (data.axes[1]))
            print("Joy2 - Right horz:  %2.3f" % (data.axes[2]))
            print("Joy3 - Right vert:  %2.3f" % (data.axes[3]))
            print("Joy4 - Pad   horz:  %2.3f" % (data.axes[4]))   # Pad is one of: +1.0, 0.0, -1.0
            print("Joy5 - Pad   vert:  %2.3f" % (data.axes[5]))


        if BAL_PID_MODE:
            if not top_right_trigger:
                self.bal_joy_speed_cmd = 0.0
                self.bal_joy_turn_cmd = 0.0
                if BAL_DOUBLE_WHEEL:
                    pub_right_leg_ankle_rotate.publish(self.bal_ankle_neutral_position)
                    pub_left_leg_ankle_rotate.publish( self.bal_ankle_neutral_position)
            else:    
                # Motors enabled
                self.bal_joy_speed_cmd =  data.axes[3]  * 1.0 # +/1 1.0 max  
                self.bal_joy_turn_cmd = data.axes[2]    * 1.0

            if BAL_DOUBLE_WHEEL:
                # if self.enable_balancing:
                # Raise or lower the back wheel to tilt the robot, making it move!
                #if self.enable_balancing and not self.robot_fall_detected:
                #    pub_right_leg_ankle_rotate.publish(self.bal_ankle_neutral_position + (self.bal_joy_speed_cmd / 10.0))
                #    pub_left_leg_ankle_rotate.publish( self.bal_ankle_neutral_position + (self.bal_joy_speed_cmd / 10.0))
                pass

        
        elif WHEEL_MOTOR_MODE:

            if not top_right_trigger:
                # Stop moving (ramp down to stop as needed)
                print("Speed: Stop")
                self.bal_joy_speed_cmd = 0
                self.bal_joy_turn_cmd = 0

            else:    
                # Motors enabled
                self.bal_joy_speed_cmd =  data.axes[3]   # range: +/- 1.0 max  
                self.bal_joy_turn_cmd = data.axes[2]    

        
        else:
            # WALKING MODE

            # Only enable joystick update when trigger held. Latches when released!
            self.left_joy_enable = top_left_trigger
            if self.left_joy_enable:
                self.left_height_joystick = data.axes[1] 

            # Only enable joystick update when trigger held. Latches when released!           
            # if top_right_trigger:   
            self.right_forward_joystick = data.axes[3] # right vert
            self.right_turn_joystick = data.axes[2] ###PID### * 10  # right horz, MM


        # Pad
        #if data.axes[5] > 0: # Pad Vert buttons
            # do something
        #if data.axes[5] < 0:
            # do something
        #if data.axes[4] > 0: # Pad Horz buttons
            # do something
        #if data.axes[4] < 0:
            # do something

    # End of Joystick handler 

    
    def slider01_cb(self, value):
        self.slider01 = float(value.data)
        
    def slider02_cb(self, value):
        self.slider02 = float(value.data)
        
    def slider03_cb(self, value):
        self.slider03 = float(value.data)
        
    def slider04_cb(self, value):
        self.slider04 = float(value.data)
        
    def slider05_cb(self, value):
        self.slider05 = float(value.data)
        
    def slider06_cb(self, value): # SERVO SPEED
    
        # TODO TODO SERVO SPEED SLIDER IGNORED 
        return
        
        if float(value.data) < 20.0: # Yeah, this really happened and I spent 3 days debugging it
            print("************** ERROR! ATTEMPT BY GUI TO SET SERVO SPEED TO ZERO! **************")
            return 
        self.slider06 = float(value.data)
        self.walking_servo_speed = self.slider06 / 100.0
        self.set_servo_speed(self.walking_servo_speed) 
            

    def slider07_cb(self, value): # IMU Tilt offset (handled in IMU callback)
        self.slider07 = float(value.data)       
        
    def slider08_cb(self, value):  # Balance PID P
        self.bal_kp = BAL_PID_DEFAULT_KP + float(value.data) / 50.0 # 0 to 10.0
        
    def slider09_cb(self, value): # Balance PID I
        self.bal_ki = BAL_PID_DEFAULT_KI + float(value.data) / 50.0 # 0 to 10.0
        
        ###self.slider09 = float(value.data)

    def slider10_cb(self, value):# Balance PID D
        self.bal_kd = BAL_PID_DEFAULT_KD + float(value.data) / 50.0 #   0 to +/-10.0
        ###self.slider10 = float(value.data)



    def slider11_cb(self, value): ###PID### IMU Pitch offset (handled in IMU callback)
        self.slider11 = float(value.data)

    def slider12_cb(self, value):
        self.slider12 = float(value.data)

    def walk_mode_cb(self, value):
        self.set_walk_mode(int(value.data))

    def set_walk_mode(self, mode):
        if mode < MODE_STOP:   
            print("WALK MODE setting POSE")
            self.walking_enabled = False

            if (BAL_PID_MODE):
                self.imu_pitch_hardware_offset_degrees = BAL_DEFAULT_IMU_OFFSET
                # Make sureback wheels are down for support
                pub_right_leg_ankle_rotate.publish(self.bal_ankle_neutral_position)
                pub_left_leg_ankle_rotate.publish(self.bal_ankle_neutral_position)

            # move into pose position
            self.set_pose(mode)
            rospy.sleep(2.0)

            # TODO - IS THIS NEEDED? Make new poses? Position head for walking
            #pub_neck_raise.publish(-0.80)
            #pub_head_tilt.publish(-0.70)
            #pub_head_sidetilt.publish(0.00)
        
        elif mode == MODE_STOP: 
            print("WALK MODE set to STOP")

            if BAL_PID_MODE or WHEEL_MOTOR_MODE:
                self.enable_balancing = False
                # Stop motors
                print("Motors: Stop")
                self.pub_wheel_cmd_right.publish(0)
                self.pub_wheel_cmd_left.publish(0)
                # put the back wheels down for sitting support
                pub_right_leg_ankle_rotate.publish(self.bal_ankle_neutral_position)
                pub_left_leg_ankle_rotate.publish(self.bal_ankle_neutral_position)
                
            else:
                self.walking_enabled = False
                self.set_servo_speed(0.5) # Set servos speed
    
        elif mode == MODE_WALK: 
            print("WALK MODE set to WALK")
            if (BAL_PID_MODE):
                self.bal_error_sum = 0.0 # clear error to avoid jerk on startup
                self.enable_balancing = True
                self.robot_fall_detected = False # reset fall detector
                if not BAL_DOUBLE_WHEEL:
                    # Move the idler wheels up to allow full balancing
                    pub_right_leg_ankle_rotate.publish(-0.95)
                    pub_left_leg_ankle_rotate.publish(-0.95)
                                  
            elif not WHEEL_MOTOR_MODE:
                self.initialize_walking()
                self.walking_enabled = True
                 
        else:
            print("******* ERROR - BAD WALK MODE ********")




    def print_debug(self, tag, elapsed_time):
       rospy.loginfo("%s Turn: % 2.1f Rx: % 2.1f Lx: % 2.1f high: % 2.2f  delay: % 2.2f imu:%+6.2f   T1: %3d  T2: %3d  T3: %3d  milis: %d" % 
            (tag, self.right_turn_joystick, self.right_x, self.left_x, self.step_height,  self.timer_delay, self.imu_roll, self.timer1, 
            self.timer2, self.timer3, elapsed_time))    
            
    
    def initialize_walking(self):    
        # Taking first steps after not walking.
        # first step from standing re-initialize everyting

        self.first_step = True
        self.set_servo_speed(self.walking_servo_speed)
        # Position head for walking
        #pub_neck_raise.publish(-0.80)
        #pub_head_tilt.publish(-0.70)
        #pub_head_sidetilt.publish(0.00)
        
        #self.previousStepMillis = self.currentMillis
        self.previousRightStepMillis = self.currentMillis
        self.previousLeftStepMillis = self.currentMillis
        self.pidMillis = self.currentMillis  
        self.step_flag = 0
        self.left_step_flag = 0
        self.right_step_flag = 0
        self.startMillis = rospy.get_time() * 1000.0 # convert float seconds to ms
        self.height_increase = 0.0
        self.cycle_count = 0

        self.imu_cycle = 1
        self.imu_pos_peak = 0.0
        self.imu_neg_peak = 0.0
        self.last_imu_pos_peak = 0.0
        self.last_imu_neg_peak = 0.0

        self.step_height = self.default_step_height            
        start_up = self.mid_height + (self.step_height / 2)
        self.left_z = start_up
        self.right_z = start_up
        self.left_x = 0      
        self.right_x = 0
        self.right_turn_joystick = 0
        self.right_forward_joystick = 0
        print("")
        print("******************* START WALK CYCLE ********************")
        
        self.loop_time = rospy.Time.now()
        
    # ======================================================================================================================
    # MAIN LOOP
    
    def timer_loop(self, event=None):
        # called every 20 ms to process events - UPDATE - fast servos run at ~16.6 ms, so 20ms loop possible
        
        #print("Timer Loop!")
        #print("")
        #print("current time is: ", rospy.get_time())

        self.currentMillis = rospy.get_time() * 1000.0 # convert float seconds to ms
        # print("****************** self.currentMillis = ", self.currentMillis)
        #print("Mainloop: imu roll = %2.3f, pitch = %2.3f, yaw = %2.3f," % (self.imu_roll, self.imu_pitch, self.imu_yaw))

                  

        if (BAL_PID_MODE):
            pid_sample_time = self.currentMillis - self.pidMillis
            self.pidMillis = self.currentMillis


            # Calculate motor speed
            # TODO Ramp from past to current value as needed
            # TODO first ramp ankle servos, then when at speed, reduce ankle slope and replace with bal_target_angle offset
            self.bal_target_angle = self.bal_motor_ramp()
            
            #max_angle = 6.0 # degrees lean for full speed
            #self.bal_target_angle = 0 # self.bal_joy_speed_cmd * max_angle
            #right_speed = self.clamp((forward_speed + turn_speed), 1.0)  
            #left_speed = self.clamp((forward_speed - turn_speed), 1.0)
            
            self.bal_current_angle = self.imu_pitch_compensated         
            current_error = self.bal_current_angle - self.bal_target_angle
            
            # For EB with double wheels, add IMU dead zone to avoid hunting near zero
            current_error = self.deadband(current_error, self.slider03/10) # 1.2
            
            tmp_sum = self.bal_error_sum + current_error
            self.bal_error_sum = self.constrain(tmp_sum, -50.0, 50.0) # Limit correction history
            # calculate output from P, I and D values
            pcalc = (self.bal_kp * current_error)
            icalc = (self.bal_ki * self.bal_error_sum * (pid_sample_time / 100.0))
            dcalc = (self.bal_kd * (self.bal_current_angle-self.bal_prev_angle) / (pid_sample_time/100))
            motor_cmd = (pcalc + icalc + dcalc) / 10.0
            # prevent slow drift with Motor Deadzone
            self.bal_motor_command = self.deadband(motor_cmd, self.slider04/10) # 0.4

            
            #self.bal_motor_command = (self.bal_kp * current_error) + (self.bal_ki * self.bal_error_sum * pid_sample_time) - (self.bal_kd * (self.bal_current_angle-self.bal_prev_angle) / pid_sample_time)
            
            self.bal_prev_angle = self.bal_current_angle

            # Publish PID info for graphing
            self.pub_imu_pitch.publish(self.bal_current_angle * 10)
            self.pub_pid_pcalc.publish(pcalc * 10.0)
            self.pub_pid_icalc.publish(icalc * 10.0)
            self.pub_pid_dcalc.publish(dcalc * 10.0)
            self.pub_pid_error.publish(current_error * 10)
            self.pub_pid_error_sum.publish(self.bal_error_sum * 10 )

            left_speed =  self.bal_motor_command # + (self.bal_joy_speed_cmd * 5.0)
            right_speed = self.bal_motor_command # + (self.bal_joy_speed_cmd * 5.0)
            dbg_elapsed = (self.currentMillis - self.previousStepMillis)
            #rospy.loginfo("PID: Time:% 2.1f, IMU:% 2.1f, Err:% 2.3f, L = % 2.3f, R = % 2.3f" % 
            #    (dbg_elapsed, self.bal_current_angle, current_error, left_speed, right_speed))
            # PRINT DEBUG INFO
            if True: # current_error != 0.0:
                print("Pc:% 2.2f Ic:% 2.2f Dc:% 2.2f Err:% 2.1f ErrSum:% 2.1f M:% 2.1f | P:% 2.2f I:% 2.2f D:% 2.2f | IMU:%2.1f Time:% 2.1f" % 
                   (pcalc, icalc, dcalc, current_error, self.bal_error_sum, left_speed, self.bal_kp, 
                   self.bal_ki, self.bal_kd, self.bal_current_angle, pid_sample_time ))

            # rospy.loginfo("Pc:% 2.2f, Ic:% 2.2f, Dc:% 2.2f" % (pcalc, icalc, dcalc))

            # Publish PID info for graphing
            self.pub_motor_command.publish(left_speed * 10)

            if (self.bal_current_angle < -40.0) or (self.bal_current_angle > 40.0):
                # stop the motors if robot has fallen over
                self.pub_wheel_cmd_right.publish(0.0)
                self.pub_wheel_cmd_left.publish(0.0)
                self.robot_fall_detected = True
                
            #elif self.robot_fall_detected:
                # wait until fall is reset to enable motors
                #print("DBG: ANKLE EFFORT = %03.3f" % self.right_ankle_effort)
                #if self.left_ankle_effort > 0.02: # user put robot on the ground
                #    self.bal_error_sum = 0.0 # clear sum error to avoid jerk on startup
                #    self.robot_fall_detected = False # Allow motors to move                 

            elif self.enable_balancing and not self.robot_fall_detected:
                # SEND WHEEL COMMANDS!
                turn = self.bal_joy_turn_cmd * 0.5 # Dynamixel is 6.2 max 
                self.pub_wheel_cmd_right.publish(right_speed + turn) 
                self.pub_wheel_cmd_left.publish((left_speed - turn) * -1.0) # reversed motor
                

            # ********************************************************
            # auto-tune IMU. imu_pitch_offset gets applied in IMU callback
            if (self.bal_target_angle == 0) and (abs(self.bal_motor_command) < 0.5): 
                #self.imu_pitch_offset = self.imu_pitch_tune + (self.imu_pitch / 10.0)
                
                # Ramp imu offset until it cancels any error      
                if self.imu_pitch_offset < self.imu_pitch:
                    self.imu_pitch_offset = min((self.imu_pitch_offset + .01), self.imu_pitch) 
                    
                elif self.imu_pitch_offset > self.imu_pitch:
                    self.imu_pitch_offset = max((self.imu_pitch_offset - .01), self.imu_pitch) 


                if False:
                    print("IMU RAMP: offset=% 2.2f  imu=% 2.2f  corrected=% 2.2f" % 
                    (self.imu_pitch_offset, self.imu_pitch, (self.imu_pitch - self.imu_pitch_offset)))



            self.previousStepMillis = self.currentMillis
                
            return


        if WHEEL_MOTOR_MODE:
        


            # Calculate motor speeds, clamping to +/- 6.0 for Dynamixel MX28
            ramped_speed_command = self.wheel_motor_ramp()            
            forward_speed = ramped_speed_command * DYNAMIXEL_64_MAX_SPEED

            turn_speed = self.bal_joy_turn_cmd * DYNAMIXEL_64_MAX_SPEED * 0.25 # don't make turns too sensitive
            
            right_speed = self.clamp((forward_speed + turn_speed), DYNAMIXEL_64_MAX_SPEED)  
            left_speed = self.clamp((forward_speed - turn_speed), DYNAMIXEL_64_MAX_SPEED)

            if  left_speed != 0.0 and right_speed != 0.0:           
                print("Motors: L = %d, R = %d" % (left_speed, right_speed))
            
            self.pub_wheel_cmd_right.publish(right_speed) # reversed motor
            self.pub_wheel_cmd_left.publish(-left_speed) 



        
            return # disable walking (for now)
            

        if not self.walking_enabled: # debug moving leg joints up and down
        
            if self.left_joy_enable and (self.currentMillis - self.previousStepMillis) > 100.0: 
                # Only send servo commands when enabled, and don't flood servo queue with too many messages
                z = self.mid_height + (self.half_control_range * self.left_height_joystick) # allow leg to go +/- from mid
                x = 0 # self.right_forward_joystick * 50  # +/- < 50mm steps
                y = 0
                ## print("Test Mode: z = %2.3f, x = %2.3f imu ROLL = %3.4f" % (z, x, self.imu_roll))
                #self.pub_imu_roll.publish(self.imu_roll*10)
                #self.pub_imu_pitch.publish(self.imu_pitch*10)
                #self.pub_imu_yaw.publish(self.imu_yaw*10)
                self.kinematics(1, x, y, z)
                self.kinematics(2, x, y, z)
                self.previousStepMillis = self.currentMillis

            return


        # ###########################################################################################        
        # Normal walking mode



        # Calculate step height
        self.step_height = self.default_step_height + self.height_increase + (self.slider12 / 10.0)

        # TODO if self.cycle_count < 1: # Step is larger at first to get things going
        #     self.step_height = 12
            
        legDown = self.mid_height + (self.step_height / 2)
        legUp = self.mid_height - (self.step_height / 2)
        # print("step height = ", self.step_height)

        # ================================================================
        # TIMERS
        
        self.timer_delay = 1.17 + self.slider01/100.0  #  1.51=slow, 1.11 = fast
        #print("self.timer_delay = %4.2f" % self.timer_delay)
        
        imu_roll_scaled = self.imu_roll * (1.0 - self.slider05 / 100.0) # mul from 0 --> 2x
        self.timer1 = 300 * self.timer_delay   # step trigger time
        self.timer2 = (((abs(imu_roll_scaled*10.0)) + 50.0) * self.timer_delay) - self.slider02 # leg trigger timer
        self.timer3 = (250 - (abs(imu_roll_scaled * 3.0)))  + self.slider03 # leg step timer  ** inverted **
        if (self.timer3 < 150 or self.timer3 > 300):
            print("*** Timer3 Clip [%d]" % self.timer3)
        self.timer3 = ((np.clip(self.timer3, 150, 300)) * self.timer_delay) 

        ### X - Forward Movement Control ###
        x_forward = self.x_offset + (self.right_forward_joystick * 20) ###PID### + (self.slider11/2.0)   # TODO: 100 = +/- 100mm steps
        x_turn = self.right_turn_joystick + self.x_turn_offset - (self.slider09/5.0)

        # Disable forward movement until leaning far enough for walking to start
        # OLD: if self.imu_roll < self.x_tilt_threshold:
        if self.cycle_count < self.x_start_cycle: # This is a bit of a kludge. Use imu peak instead?
            x_forward = 0
            x_turn = 0
            self.right_forward_joystick = 0
            self.right_turn_joystick = 0
                
        elapsed_time = self.currentMillis - self.previousStepMillis
        
        # ================================================================
        # MAIN STEP TRIGGERS
        
        #imu_threshold = self.slider04 / 10.0
        #print("imu_threshold = ", imu_threshold)
        # if self.imu_roll > imu_threshold
       

        if (self.step_flag == 0) and (elapsed_time > self.timer1):
            # wait 300ms then tell right foot to move
            self.loop_time_result = rospy.Time.now() - self.loop_time
            self.loop_time = rospy.Time.now()
            print("")
            print("Loop Time: %3.2f" % self.loop_time_result.to_sec())
            self.print_debug(" Main 0:", elapsed_time)            
            self.previousStepMillis = self.currentMillis # reset timer for the next step
            self.step_flag = 1 
            self.right_step_flag = 1 # Begin right step

        elif (self.step_flag == 1) and (elapsed_time > self.timer2):
            self.print_debug(" Main 1:", elapsed_time)            
            self.previousStepMillis = self.currentMillis
            self.step_flag = 2   
            self.last_imu_pos_peak = self.imu_pos_peak
            self.imu_pos_peak = 0
            self.imu_cycle = -1

        elif (self.step_flag == 2) and (elapsed_time > self.timer1):
            print("")
            self.print_debug(" Main 2:", elapsed_time)            
            self.previousStepMillis = self.currentMillis
            self.step_flag = 3 
            self.left_step_flag = 1   # Begin left step      

        elif (self.step_flag == 3) and (elapsed_time > self.timer2):
            self.print_debug(" Main 3:", elapsed_time)            
            self.previousStepMillis = self.currentMillis
            self.step_flag = 0          
            self.first_step = False # done with first tilt
            self.last_imu_neg_peak = self.imu_neg_peak
            self.imu_neg_peak = 0
            self.imu_cycle = 1
            self.cycle_count = self.cycle_count + 1

        dbg_x_left = 0
        dbg_x_right = 0



        # ================================================================
        # RIGHT LEG STEPS

        # RIGHT LEG UP (Lean right)
        if (self.right_step_flag == 1) and (self.currentMillis - self.previousRightStepMillis > (self.timer3/2)):
            self.previousRightStepMillis = self.currentMillis
            self.right_z = legUp + self.slider10/20.0
            self.kinematics(RIGHT_LEG, self.right_x, self.right_y, self.right_z)
            self.pub_right_servo_cmd.publish(-300)
            self.print_debug("Right 1:", elapsed_time)            
            self.right_step_flag = 2

        # RIGHT LEG FORWARD
        elif (self.right_step_flag == 2) and (self.currentMillis - self.previousRightStepMillis > (self.timer3/2)):
            self.previousRightStepMillis = self.currentMillis
            self.right_x = (x_forward - x_turn)
            #print("R Up+Fwd: % 3.2f" % self.right_x)
            self.kinematics(RIGHT_LEG, self.right_x, self.right_y, self.right_z)
            # TODO pub_right_leg_hip_rotate.publish(self.hip_turn) # Rotate Hip for turns
            dbg_x_right = 50            
            self.print_debug("Right 2:", elapsed_time)            
            self.right_step_flag = 3

        # LEG DOWN
        elif (self.right_step_flag == 3) and (self.currentMillis - self.previousRightStepMillis > (self.timer3/2)):
            self.previousRightStepMillis = self.currentMillis
            self.right_z = legDown + self.slider10/20.0    
            self.kinematics(RIGHT_LEG, self.right_x, self.right_y ,self.right_z)
            self.pub_right_servo_cmd.publish(-400)
            self.print_debug("Right 3:", elapsed_time)            
            self.right_step_flag = 4

        # RIGHT LEG BACK
        elif (self.right_step_flag == 4) and (self.currentMillis - self.previousRightStepMillis > (self.timer3/2)):
            self.previousRightStepMillis = self.currentMillis
            self.right_x = (x_forward - x_turn) * -1.0
            self.kinematics(RIGHT_LEG, self.right_x, self.right_y ,self.right_z)
            # TODO pub_right_leg_hip_rotate.publish(0.0) # return hip to neutral
            # print("R Dn+Bck: % 3.2f" % self.right_x)
            dbg_x_right = -50
            self.print_debug("Right 4:", elapsed_time)            
            self.right_step_flag = 0
            
            
        # ================================================================
        # LEFT LEG STEPS

        # LEFT LEG UP (Lean left)
        if (self.left_step_flag == 1) and (self.currentMillis - self.previousLeftStepMillis > (self.timer3/2)):
            self.previousLeftStepMillis = self.currentMillis
            self.left_z = legUp - self.slider10/20.0  
            self.kinematics(LEFT_LEG, self.left_x, self.left_y ,self.left_z)     
            self.pub_left_servo_cmd.publish(400)
            self.print_debug("Left  1:", elapsed_time)            
            self.left_step_flag = 2
            
        # LEFT LEG FORWARD    
        elif (self.left_step_flag == 2) and (self.currentMillis - self.previousLeftStepMillis > (self.timer3/2)):
            self.previousLeftStepMillis = self.currentMillis
            self.left_x = (x_forward + x_turn)
            print("L Up+Fwd: % 3.2f" % self.left_x)
            dbg_x_left = 50
            self.kinematics(LEFT_LEG, self.left_x, self.left_y ,self.left_z)
            # TODO pub_left_leg_hip_rotate.publish(-self.hip_turn) # Rotate Hip for turns
            #self.pub_left_fwd.publish(250 + 100)
            self.print_debug("Left  2:", elapsed_time)            
            self.left_step_flag = 3

        # LEG DOWN
        elif (self.left_step_flag == 3) and (self.currentMillis - self.previousLeftStepMillis > (self.timer3/2)):
            self.previousLeftStepMillis = self.currentMillis
            self.left_z = legDown - self.slider10/20.0    
            self.kinematics(LEFT_LEG, self.left_x, self.left_y ,self.left_z)     
            self.pub_left_servo_cmd.publish(300)
            self.print_debug("Left  3:", elapsed_time)            
            self.left_step_flag = 4

        # LEFT LEG BACK
        elif (self.left_step_flag == 4) and (self.currentMillis - self.previousLeftStepMillis > (self.timer3/2)):
            self.previousLeftStepMillis = self.currentMillis
            self.left_x = (x_forward + x_turn) * -1.0
            print("L Dn+Bck: % 3.2f" % self.left_x)
            dbg_x_left = -50
            self.kinematics(LEFT_LEG, self.left_x, self.left_y ,self.left_z)
            # TODO pub_left_leg_hip_rotate.publish(0.0) # return hip to neutral    
            self.print_debug("Left  4:", elapsed_time)            
            self.left_step_flag = 0
            
 
        self.right_y = 0
        self.left_y = 0

        debug_pub_step_cycle = True
        if debug_pub_step_cycle:    # for graphing with rqt_plot
            # rqt_plot defaults to +/- 150, so scale these vales to be on separate "lines"
            self.pub_imu_roll.publish(self.imu_roll*10)
            #self.pub_imu_pitch.publish(self.imu_pitch*10)
            self.pub_imu_yaw.publish(self.imu_yaw*10)
                       
            self.pub_step_cycle.publish(self.step_flag*20)
            self.pub_timer2.publish(self.timer2)
            self.pub_timer3.publish(self.timer3)
            
            self.pub_left_cycle.publish(150 + self.left_step_flag*10)
            self.pub_left_up.publish(100 - (self.left_z - self.mid_height) * 2 )
            self.pub_left_fwd.publish(250 + dbg_x_left + (self.left_x * 2))

            
            self.pub_right_cycle.publish(-350 + self.right_step_flag*10 + 100)
            self.pub_right_up.publish(-300 - (self.right_z - self.mid_height) * 2 )
            self.pub_right_fwd.publish(-150 + dbg_x_right + (self.right_x * 2))
            

    #----------------------------------------------------------------------------------------------------
    def kinematics (self, leg, xIn, yIn, zIn):

        if zIn < 220 or zIn > 230:
            #print("DBG kinematics: leg: %d, x: %d, y: %d, z: %d"% (leg, xIn, yIn, zIn))
            rospy.loginfo("DBG kinematics: leg: %d, x: %d, y: %d, z: %d"% (leg, xIn, yIn, zIn))
        # left_ is LEFT!
            
        if zIn < self.min_leg_height:
            print("ERROR! Z must be a greater than min_leg_height (%d)! z = %d" % (self.min_leg_height, zIn))
            return  
        
        z = zIn # Step height
        x = xIn # Step length
        #y = yIn # Not used
          
        if z <= 0:
            print("ERROR! Z must be a postive number! z = %d" % z)
            return  
            

        target_height = self.constrain(z, self.min_leg_height, self.max_leg_height)
        # print("target_height = %d" % target_height)
        step_distance = x       

        # calculate the shoulder joint offset and new leg length based on now far the foot moves forward/backwards
        phantom_leg_length = target_height 
        step_angle = math.atan(step_distance/target_height)     # calc how much extra to add to the shoulder joint 
        # print("step angle = %1.4f rad,   %3.1f degrees" % (step_angle, np.degrees(step_angle)))        
         
        if step_angle != 0.0:
            phantom_leg_length = target_height / math.cos(step_angle)
            # print("New phantom_leg_length = ", phantom_leg_length)
        

        opp_over_hyp = (phantom_leg_length / 2.0) / self.thigh_length
        # print("opp_over_hyp = ", opp_over_hyp)
        
        half_knee_angle = math.asin( opp_over_hyp)
        # print("calculated thigh and ankle angle = %1.4f rad,   %3.1f degrees" % (half_knee_angle, np.degrees(half_knee_angle)))
        
        half_pi = math.pi/2
        # print("half pi = %f" % (half_pi) )
        
        normalized_half_knee_angle = (half_pi) - half_knee_angle
        # print("normalized to servo: thigh and ankle angle = %1.4f rad,   %3.1f degrees" % (normalized_half_knee_angle, np.degrees(normalized_half_knee_angle)))
        
        normalized_knee_angle = normalized_half_knee_angle * 2.0        
        # print("normalized to servo: knee angle = %1.4f rad,   %3.1f degrees" % (normalized_knee_angle, np.degrees(normalized_knee_angle)))

        shoulder_angle_with_step = normalized_half_knee_angle - step_angle
        # print("shoulder_angle_with_step = ", shoulder_angle_with_step)
        #ankle_angle_with_step = normalized_half_knee_angle + step_angle


        # ***** WRITE TO DYNAMIXELS *****

       
        # if not self.walking_enabled:
            # print("Moving legs in test mode.")
            # leg = 3
    
        # TODO - Robot leaning forward, for now, just add hardcoded ankle compensation
        #ankle_compensation = np.radians(-1.0)    # Number of degrees 3?

        if (leg == LEFT_LEG):      # *** PUBLISH LEFT ***
            if zIn < 220 or zIn > 230:
                print("Publishing Left leg.")
            pub_left_leg_knee_bend.publish(normalized_knee_angle)
            pub_left_leg_thigh_lift.publish(shoulder_angle_with_step * -1.0)
            #pub_left_leg_ankle_rotate.publish((ankle_angle_with_step + ankle_compensation) * -1.0)
            

            if self.walking_enabled: # Publish servo position info for graphing
                self.pub_left_knee_cmd.publish((normalized_knee_angle * 1000) - 900 )
                #self.pub_left_ankle_cmd.publish(((ankle_angle_with_step + ankle_compensation) * 2000) - 900)
                self.pub_left_thigh_cmd.publish((shoulder_angle_with_step * 2000) - 900 )


        elif (leg == RIGHT_LEG):    # *** PUBLISH RIGHT ***
            if zIn < 220 or zIn > 230:
                print("Publishing Right leg.")
            pub_right_leg_knee_bend.publish(normalized_knee_angle)
            pub_right_leg_thigh_lift.publish(shoulder_angle_with_step * -1.0)
            #pub_right_leg_ankle_rotate.publish((ankle_angle_with_step + ankle_compensation) * -1.0)

            if self.walking_enabled: # Publish servo position info for graphing
                self.pub_right_knee_cmd.publish((normalized_knee_angle * 1000) - 1900 )
                #self.pub_right_ankle_cmd.publish(((ankle_angle_with_step + ankle_compensation) * 2000) - 1900 )
                self.pub_right_thigh_cmd.publish((shoulder_angle_with_step * 2000) - 1900 )

                       
        elif (leg == 3):
            print("Leg Publishers Disabled!")

        else:
            print("ERROR! Bad LEG NUMBER! (%d)" % (leg))

        # print("------------------------")


            
        
if __name__ == '__main__':

    # capture SIGINT signal, e.g., Ctrl+C
    # signal.signal(signal.SIGINT, signal_handler)

    node = WalkEngine()
    # note that the timer is the main loop function!
    # other option is to use: while not rospy.is_shutdown():  
    # see Idle Behavior or face_detector.py: node.run()

    rospy.Timer(rospy.Duration(0.02), node.timer_loop) # 0.01 = 10ms, 0.02 = 20ms
    # node.run()
    rospy.spin()



