#!/usr/bin/env python3
# Wheel Control - Controls servos that drive wheels
# Note! python file name should not be same as package name!


import rospy
import logging
import time
import math
import sys
import signal
#from angles import fabs, shortest_angular_distance
#import numpy as np


from sensor_msgs.msg import Joy
#from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import JointState # single servo messages
from dynamixel_msgs.msg import JointStateArray
from sensor_summary_msgs.msg import SensorSummary
from simple_move_msgs.msg import SimpleMove

from std_msgs.msg import Float32
from std_msgs.msg import UInt16
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Bool

from behavior_msgs.msg import CommandState
from system_status_msgs.msg import SystemStatus

# Odometry
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Point32, Pose, Quaternion, Twist, Vector3

#from dynamixel_controllers.srv import TorqueEnable, SetServoTorqueLimit, SetSpeed
from eb_servos.set_pose import *    # for RobotPose
from eb_servos.servo_joint_list import all_servo_joints, head_joints, right_leg_joints, left_leg_joints
from eb_servos.head_servo_publishers import *
from eb_servos.leg_servo_publishers import *

#from eb_servos.standard_servo_positions import *
from eb_servos.set_servo_speed import *
from eb_servos.set_servo_torque import *
from eb_servos.usb_latency_test import *
from eb_servos.srv import ReturnJointStates

from eb_wheel_control.programmed_move import ProgrammedMove

COMMAND_TIMEOUT_SECONDS = 1.0 # if no message after this time, stop the wheels
WHEEL_MOTOR_MODE = True
DYNAMIXEL_64_MAX_SPEED = 6.762 # radians per second
DEFAULT_IMU_OFFSET = 2.2 # 5.8 #10.4 # 3.0
ANKLE_NEUTRAL_OFFSET = -0.0 # 800 # bigger = lean back


def signal_handler(signal, frame):
    global g_interrupted
    g_interrupted = True



#-------------------------------------------------------------------------------    
class WheelControl():
    def __init__(self):
        rospy.init_node('wheel_control')
        rospy.loginfo("*************** Starting Wheel Control **************")
        self.module_name = "wheel_control"

        if usb_latency_test() == False:
            rospy.logwarn("Wheel_Control: USB Latency Test failed! Servos may not respond quickly.")  
            #sys.exit() # Only critical if doing walking or wheel balancing

        
        # CONSTANTS
        self.WHEEL_RADIANS_PER_METER = 0.03
        self.imu_roll_hardware_offset_degrees = 0.0 
        self.imu_pitch_hardware_offset_degrees = DEFAULT_IMU_OFFSET

        
        # Variables        

        self.speed_cmd = 0.0
        self.turn_cmd = 0.0
        self.joystick_in_use = False
        self.command_timeout_start = None
        
        self.robotpose = RobotPose("wheel_control")
        self.last_motor_position_right = 0.0
        self.last_motor_position_left = 0.0
        self.odom_radians_right = 0.0
        self.odom_radians_left = 0.0
        self.battery_reported_level = 0 # minutes of battery remaining (assume no battery)
        self.speech_client = None
        self.imu_pitch_offset = 0.0
        self.imu_pitch_compensated = 0.0
        
        self.imu_roll = 0.0     # positive = roll left           
        self.imu_pitch = 0.0    # positive = pitch forward           
        self.imu_yaw = 0.0      # positive = yaw left
        self.speed_ramp = 0.0
        self.turn_ramp = 0.0
        self.fast_ramp_enabled = False
        self.wheel_torque_is_on = False
        self.last_speed_cmd = 0.0

        # collision avoidance
        self.object_avoidance_enabled = True 
        self.collision_prevention_enabled = True       
        self.collision_slow_distance = 0.400 # 0.350 # slow down if center object closer than this  TODO: 0.5: # meters
        self.collision_stop_distance_front = 0.200 # stop if center object closer than this
        self.collision_stop_distance_rear = 0.350 # stop if center object closer than this
        self.avoidance_distance = 1.000 # slow down and/or turn if object closer than this
        self.COLLISION_TURN_SPEED = 0.3
        ##self.handling_collision = False       
        self.handling_avoidance = False       
        self.object_front_status = ''
        self.object_rear_status = ''
        self.status_update_counter = 0
        self.sensor_summary = None                              
                              
        self.programmed_move = ProgrammedMove(self.collision_slow_distance, 
            self.collision_stop_distance_front, self.collision_stop_distance_rear)


        # PUBLISHERS
        self.simple_wheel_odom_pub = rospy.Publisher('/simple_odom', Float64, queue_size=50)
        self.status_pub = rospy.Publisher('/system_status', SystemStatus, queue_size=6)
        #self.simple_right_wheel_odom_pub = rospy.Publisher("/simple_odom_right", Float64, queue_size=50)
        #self.simple_left_wheel_odom_pub = rospy.Publisher("/simple_odom_left", Float64, queue_size=50)
        #self.odom_broadcaster = tf.TransformBroadcaster()

        # Note that servo publishers are defined in includes
        self.pub_peek_cmd =     rospy.Publisher('/head_peek',   Bool, queue_size=1) 
        self.behavior_cmd_pub = rospy.Publisher('/behavior/cmd', CommandState, queue_size=2)
        self.pub_wheel_cmd_right = rospy.Publisher('/right_wheel_motor_joint/command', Float64, queue_size=1)
        self.pub_wheel_cmd_left  = rospy.Publisher('/left_wheel_motor_joint/command',  Float64, queue_size=1)


        # SUBSCRIBERS
 
        # get imu updates from Arduino BNO086
        imu_sub = rospy.Subscriber("/imu_orientation", Point32, self.imu_callback)

        # monitor object avoidance sensors
        sensor_sub = rospy.Subscriber("/sensor_fusion", SensorSummary, self.sensor_callback)

        # Movement subscribers
        joystick_sub = rospy.Subscriber("/joy", Joy, self.joy_cb) # Standard Logitech / Xbox joystick
        phone_sub = rospy.Subscriber("/phone_joy", Joy, self.phone_joy_cb) # From Bluetooth Phone
        twist_sub = rospy.Subscriber("/cmd_vel", Twist, self.twist_cb) # commands from other modules
        simple_move_sub = rospy.Subscriber("/simple_move", SimpleMove, self.simple_move_cb) # move commands from other modules

        # Odometry from wheel servos
        right_wheel_servo_sub = rospy.Subscriber('/right_wheel_motor_joint/state', JointState, self.right_wheel_cb) 
        left_wheel_servo_sub  = rospy.Subscriber('/left_wheel_motor_joint/state',  JointState, self.left_wheel_cb) 
        
      
        # Final Initialization
        self.pub_wheel_cmd_right.publish(0)
        self.pub_wheel_cmd_left.publish(0)



        rospy.loginfo("Wheel Control: Ready.")       
        #------------------------------------ Init end --------------------------------------        
        

    def __del__(self):
        rospy.loginfo("%s:__del__ called on exit" % (self.module_name))

    def clamp(self, value, max_value): # clamp between pos and neg max_value
        return max(min(value, max_value), -max_value)

    def send_status_update(self, item, status):
        # Send status update system monitor
        status_msg = SystemStatus()
        status_msg.item = item
        status_msg.status = status
        self.status_pub.publish(status_msg)

    def set_speed(self, speed):
        self.speed_cmd = speed
        self.command_timeout_start = time.time() # reset the timeout

    def set_turn(self, turn):
        self.turn_cmd = turn   
        self.command_timeout_start = time.time() # reset the timeout

    

    def sensor_callback(self, data):
        # rospy.loginfo("%s:got sensor data" % (self.module_name))

        self.sensor_summary = data

        # pass sensor data to local callback for programmed moves
        self.programmed_move.sensor_callback(data) 

        '''
        data.nearest_object_front = self.max_laser_max_range
        data.nearest_object_front_right = self.max_laser_max_range
        data.nearest_object_front_left = self.max_laser_max_range
        data.nearest_object_side_right = self.max_laser_max_range
        data.nearest_object_side_left = self.max_laser_max_range
        data.nearest_object_rear = self.max_laser_max_range
        data.clear_path_gap_direction = 0.0
        data.clear_path_gap_width = self.max_laser_max_range
        data.avoidance_range_used = self.avoid_range
        '''
        object_front_status = 'OK'
        object_rear_status = 'OK'

        # ----------------------------------------------------------------------
        # UPDATE DASHBOARD        
        # Update status dashboard - Front Sensors
        if data.nearest_object_front < self.collision_stop_distance_front:
            object_front_status = 'STOP'
        elif data.nearest_object_front < self.collision_slow_distance:
            object_front_status = 'SLOW'
        elif data.nearest_object_front < self.avoidance_distance:
            object_front_status = 'AVOID'
        
        # Update status dashboard - Rear Sensors
        if data.nearest_object_rear < self.collision_stop_distance_rear:
            object_rear_status = 'STOP'
        elif data.nearest_object_rear < self.collision_slow_distance:
            object_rear_status = 'SLOW'

        # send update if status changes, or periodically to assure updates are reported
        self.status_update_counter = self.status_update_counter +1
        if self.status_update_counter > 50:  # about every 5 seconds
            self.status_update_counter = 0       
            self.send_status_update('OBJECT_FRONT', object_front_status)
            self.send_status_update('OBJECT_REAR', object_rear_status)

        
    def collision_override(self, requested_speed):
        # Collision prevention under manual control (non-programmed moves)
        # If no problem, return the requested speed, else return overridden speed

        return_speed = requested_speed    

        if requested_speed == 0.0:
            # don't do anything if the robot is stopped with no command to move
            return return_speed

        if self.sensor_summary == None:
            rospy.loginfo("%s:collision_override: No Sensor_summary data received!" % (self.module_name))
            return return_speed

        if not self.collision_prevention_enabled:
            rospy.loginfo("%s:collision_override: collision_prevention disabled." % (self.module_name))
            return return_speed


        # slow down if object in avoidance range
        # TODO ?

        rospy.loginfo("%s:DBG collision_override: Front = %1.2f, Rear = %1.2f" % (self.module_name, 
            self.sensor_summary.nearest_object_front, self.sensor_summary.nearest_object_rear))

        # Stop if object in collision range       
        if (requested_speed > 0.0 and 
            self.sensor_summary.nearest_object_front < self.collision_stop_distance_front):
            # Moving forward and object detected ahead
            rospy.logwarn('%s: Object Ahead! Stopping!' % (self.module_name))
            return_speed = 0.0

        elif (requested_speed < 0.0 and 
            self.sensor_summary.nearest_object_rear < self.collision_stop_distance_rear):
            # Moving backward and object detected behind
            rospy.logwarn('%s: Object Behind! Stopping!' % (self.module_name))
            return_speed = 0.0

        return return_speed



    
    def imu_callback(self, data):
        #rospy.loginfo("%s:got imu: ", data % (self.module_name)) 
        self.imu_roll = data.x # + self.imu_roll_hardware_offset_degrees + (self.slider07/10.0)
        self.imu_pitch = data.y #+ self.imu_pitch_hardware_offset_degrees + (self.slider11/100.0)
        self.imu_pitch_compensated = self.imu_pitch
        if False: # abs(self.imu_pitch) < 2.0:
            self.imu_pitch_compensated = self.imu_pitch - self.imu_pitch_offset # auto-tuned IMU
        self.imu_yaw = data.z
        
        self.programmed_move.imu_callback(data) # local callback for programmed moves


    # Wheel callbacks: Get wheel servo updates to calculate odometry

    def right_wheel_cb(self, data):
        #rospy.loginfo("%s: Right Wheel Callback" % (self.module_name))
        self.odom_update("right", data.current_pos, self.last_motor_position_right, self.odom_radians_right) # position in radians
        #velocity = data.velocity
        # Right callback always comes first, so just do the internal update. Publish during left wheel motor update
        
        
    def left_wheel_cb(self, data):
        # Note that left wheel is reversed, so odom is negative
        #rospy.loginfo("%s: Left Wheel Callback" % (self.module_name))
        self.odom_update("left", data.current_pos, self.last_motor_position_left, self.odom_radians_left) # position in radians
        #velocity = data.velocity

        # Calculate odometry and publish
        # Average the two wheel values. Note that left wheel is reversed (so we subtract the negative, instead of adding)        
        motor_odom_average = (self.odom_radians_right - self.odom_radians_left) / 2.0 

        # convert to meters for publishing odom values
        motor_odom_meters = motor_odom_average / 8.125 #self.WHEEL_RADIANS_PER_METER
        #motor_odom_meters_right = self.odom_radians_right / 8.125 #self.WHEEL_RADIANS_PER_METER
        #motor_odom_meters_left = self.odom_radians_left / 8.125 #self.WHEEL_RADIANS_PER_METER
        #rospy.loginfo("%s:DBG: Simple Odom Meters:      % 3.4f" % motor_odom_meters % (self.module_name))

        # Publish Odometry!
        self.simple_wheel_odom_pub.publish(motor_odom_meters)       
        self.programmed_move.odom_callback(motor_odom_meters) # local callback for programmed moves

    def odom_update(self, wheel_side, new_position, last_position, last_odom_radians):
        
        motor_pos_delta = 0.0
        if last_position != 0.0000:  # Handle startup case
            motor_pos_delta = new_position - last_position
        
        # Handle wrap around from 6.2 radians (2PI) to zero, looking for value greater than half circle
        if motor_pos_delta > math.pi:
            motor_pos_delta = motor_pos_delta - (math.pi * 2.0)
        elif motor_pos_delta < -math.pi:
            motor_pos_delta = motor_pos_delta + (math.pi * 2.0)
           
        # Keep track of how much the wheel has moved since ROS launch 
        new_odom_radians = last_odom_radians + motor_pos_delta          

        if False: #motor_pos_delta != 0.0:
            print("%s wheel values:" % wheel_side)
            print("    Motor Delta:      % 3.4f" % motor_pos_delta)
            print("    New odom radians: % 3.8f"  % new_odom_radians)

        # Update state variables    
        if wheel_side == "right":
            self.last_motor_position_right = new_position
            self.odom_radians_right = new_odom_radians    

        elif wheel_side == "left":
            self.last_motor_position_left = new_position
            self.odom_radians_left = new_odom_radians    

        else:
            rospy.logwarn("%s:odom_update: BAD WHEEL SIDE" % (self.module_name))

    def twist_cb(self, msg):
        # Accept move commands from other modules. Joystick can override.
        #rospy.loginfo("%s: Received twist /cmd_vel: speed: %2.2f, turn: %2.2f" % (self.module_name, msg.linear.x, msg.angular.z))
        #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

        if not self.joystick_in_use:
            # only accept external commands if joystick is not overriding
            self.set_speed(msg.linear.x)           
            self.set_turn(msg.angular.z)          


    def simple_move_cb(self, msg):
        # Accept programmed move commands from other modules. Joystick can override.
        rospy.loginfo("%s:simple_move: move_amount: %2.2f, move_speed: %2.2f turn_amount: %2.2f, turn_speed: %2.2f" % 
            (self.module_name, msg.move_amount, msg.move_speed, msg.turn_amount, msg.turn_speed))
        
        (success, program_speed, program_turn) = self.programmed_move.begin(msg, self.collision_prevention_enabled)
        if success and not self.joystick_in_use:
            # begin move if joystick is not overriding
            self.set_speed(program_speed)           
            self.set_turn(program_turn)          


        
    
    def phone_joy_cb(self, data): 
        # Bluetooth Phone pretends to be left stick, while xbox controller uses the right stick
        rospy.loginfo("%s:got phone joystick callback" % (self.module_name))

        # Joystick messages from phone tilt
           
        top_left_trigger = False 
        top_right_trigger = False
        bottom_right_trigger = False
        
        top_left_trigger = data.buttons[4] # This is passed in by the arduino with a valid command  
        top_right_trigger = data.buttons[5]  
        bottom_right_trigger = data.buttons[7] 

        debug_joy_buttons = False
        if debug_joy_buttons:
            rospy.loginfo("%s:Joy button command summary:" % (self.module_name))
            rospy.loginfo("%s:  top_left_trigger =     ", top_left_trigger % (self.module_name))
            rospy.loginfo("%s:  top_right_trigger =    ", top_right_trigger % (self.module_name))
            rospy.loginfo("%s:  bottom_right_trigger = ", top_right_trigger % (self.module_name))

        # Joystick inputs (+1.0 to -1.0) Up/Left is positive
        debug_joy_axis = False
        if debug_joy_axis:
            # WARNING: If joystick does not match this, make sure the "Mode" light is OFF!
            #rospy.loginfo("%s:Raw Data Axis", data.axes)
            rospy.loginfo("%s:Joy0 - Left  horz:  %2.3f" % (data.axes[0]) % (self.module_name))
            rospy.loginfo("%s:Joy1 - Left  vert   %2.3f" % (data.axes[1]) % (self.module_name))
            rospy.loginfo("%s:Joy2 - Right horz:  %2.3f" % (data.axes[2]) % (self.module_name))
            rospy.loginfo("%s:Joy3 - Right vert:  %2.3f" % (data.axes[3]) % (self.module_name))


        # bottom right trigger enables faster speeds then normal
        #self.joystick_in_use = top_right_trigger # overrides other modules

        # Arduino passes top_left_trigger == True with a valid command from the phone
        if not top_left_trigger: 
            # Stop moving (ramp down to stop if joystick is only module moving the robot)
            rospy.loginfo("%s:Speed: Stop" % (self.module_name))
            self.speed_cmd = 0
            self.turn_cmd = 0

        else:    
            # Motors enabled
            rospy.loginfo("%s:Sending Speed and Turn" % (self.module_name))
            self.set_speed(data.axes[1])   # range: +/- 1.0 max  
            self.set_turn(data.axes[0])   
            rospy.loginfo("%s:Joystick Speed: %1.3f, Turn: %1.3f" % (self.module_name, self.speed_cmd, self.turn_cmd ))

    # End of phone Joystick handler 

        
           
    # =============================== JOYSTICK CALLBACK =================================
    def joy_cb(self, data):
        # Joystick messages
        # Only one button supported at a time (I don't think this is true anymore?)
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
                print("X Button - Blue") # peek
            if data.buttons[1] == 1:
                print("A Button - Green") # AI Mode
            if data.buttons[2] == 1:
                print("B Button - Red") # stop
            if data.buttons[3] == 1:
                print("Y Button - Yellow") # Manual Mode and Hello
            if data.buttons[4] == 1:
                print("Left Top trigger")
            if data.buttons[5] == 1:
                print("Right Top trigger")
            if data.buttons[6] == 1:
                print("Left Bottom Trigger")
            if data.buttons[7] == 1:
                print("Right Bottom trigger")
            if data.buttons[8] == 1:
                print("Back Button") # sleep
            if data.buttons[9] == 1:
                print("Start Button") # wake
            if data.buttons[10] == 1:
                print("Button 10?")
            if data.buttons[11] == 1:
                print("Button 11?")

            print("PAD TEST: 5 = ", data.axes[5])
            print("          4 = ", data.axes[4])


        # handle buttons

        if data.buttons[0] == 1:         # X - Blue: PEEK! - robot peeks head up while sleeping
            rospy.loginfo("wheel_control: Joystick Cmd: PEEK") # Only works in Sleep Mode
            self.pub_peek_cmd.publish(True)

        elif data.buttons[1] == 1:       # A - Green:  AI Mode
            rospy.loginfo("wheel_control: Joystick Cmd: AI_MODE")
            msg = CommandState()
            msg.commandState = 'AI_MODE'
            msg.param1 = 'TRUE'
            msg.param2 = ''
            self.behavior_cmd_pub.publish(msg)
            
        elif data.buttons[2] == 1:         # B - Red: Stop
            rospy.loginfo("wheel_control: Joystick Cmd: STOP, Back to IDLE") 
            self.speed_cmd = 0
            self.turn_cmd = 0
            msg = CommandState()
            msg.commandState = 'IDLE'
            msg.param1 = ''
            msg.param2 = ''
            self.behavior_cmd_pub.publish(msg)
            
        elif data.buttons[3] == 1:         # Y - Yellow: Command Mode
            rospy.loginfo("wheel_control: Joystick Cmd: AI_MODE -> FALSE (command mode") 
            self.speed_cmd = 0
            self.turn_cmd = 0
            msg = CommandState()
            msg.commandState = 'AI_MODE' #TODO
            msg.param1 = 'FALSE'
            msg.param2 = ''
            self.behavior_cmd_pub.publish(msg)
            
        elif data.buttons[8] == 1:         # back - Sleep
            rospy.loginfo("wheel_control: Joystick Cmd: SLEEP") 
            msg = CommandState()
            msg.commandState = 'SLEEP'
            msg.param1 = ''
            msg.param2 = ''
            self.behavior_cmd_pub.publish(msg)
        
        elif data.buttons[9] == 1:         # start - Wake
            rospy.loginfo("wheel_control: Joystick Cmd: WAKE") 
            msg = CommandState()
            msg.commandState = 'WAKE'
            msg.param1 = ''
            msg.param2 = ''
            self.behavior_cmd_pub.publish(msg)


        # Handle Pad - If this does not work, MAKE SURE green "mode" light is OFF on the controller!
        if data.axes[5] > 0: # Pad Vert buttons
            rospy.loginfo("wheel_control: Joystick Keypad: POSE UP") 
            msg = CommandState()
            msg.commandState = 'POSE' 
            msg.param1 = 'UP' # move up one pose
            msg.param2 = ''
            self.behavior_cmd_pub.publish(msg)
            
        elif data.axes[5] < 0:
            rospy.loginfo("wheel_control: Joystick Keypad: POSE DOWN") 
            msg = CommandState()
            msg.commandState = 'POSE' 
            msg.param1 = 'DOWN' # move up one pose
            msg.param2 = ''
            self.behavior_cmd_pub.publish(msg)
            
        if data.axes[4] > 0: # Pad Horz buttons
            rospy.loginfo("wheel_control: Joystick Keypad: INTRO") 
            msg = CommandState()
            msg.commandState = 'INTRO' 
            msg.param1 = '' 
            msg.param2 = ''
            self.behavior_cmd_pub.publish(msg)
            
        if data.axes[4] < 0:
            rospy.loginfo("wheel_control: Joystick Keypad: AVOIDANCE")  # was JOKE
            # TODO? self.collision_prevention_enabled
            if self.object_avoidance_enabled:
                rospy.loginfo("wheel_control: AVOIDANCE DISABLED")  
                self.object_avoidance_enabled = False
            else:
                rospy.loginfo("wheel_control: AVOIDANCE ENABLED")  
                self.object_avoidance_enabled = True

        # Handle Joystick
            
        top_left_trigger = False
        top_right_trigger = False
        bottom_right_trigger = False
        
        top_left_trigger = data.buttons[4]  
        top_right_trigger = data.buttons[5]  
        bottom_right_trigger = data.buttons[7] 


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


        # bottom right trigger enables faster speeds then normal
        self.fast_ramp_enabled = bottom_right_trigger
        self.joystick_in_use = top_right_trigger # overrides other modules
            

        if not top_right_trigger:
            # Stop moving (ramp down to stop if joystick is only module moving the robot)
            rospy.loginfo("Wheel Control: Joystick Speed: Stop")
            self.speed_cmd = 0
            self.turn_cmd = 0

        else:    
            # Motors enabled
            self.set_speed(data.axes[3])  # range: +/- 1.0 max  
            self.set_turn(data.axes[2])    
            rospy.loginfo("Wheel Control: Joystick Speed: %1.3f, Turn: %1.3f" % (self.speed_cmd, self.turn_cmd ))


    # End of Joystick handler 

    # ======================================================================================================================
    # Motor Ramps    
    def wheel_motor_speed_ramp(self):
        # called each time through the fast loop. Speed command is -1.0 to +1.0
        if self.fast_ramp_enabled:
            ramp_speed_seconds = 1.5 # time for full ramp from stop to max speed
        else:
            ramp_speed_seconds = 4.0 # time for full ramp from stop to max speed

        ramp_rate = 0.02 / ramp_speed_seconds # 0.02 How much to step each time through the 20ms loop (50 times / second)
        speed_cmd =  self.speed_cmd      


        # TODO - lean servos when accelerating?            
        #self.last_speed_cmd = self.speed_cmd
        
        speed_delta = speed_cmd - self.speed_ramp
        if False: #speed_delta !=0:
            pitch_radians = speed_delta / 10.0
            pub_right_leg_ankle_rotate.publish(self.ankle_neutral_position + pitch_radians)
            pub_left_leg_ankle_rotate.publish( self.ankle_neutral_position + pitch_radians)


        # Ramp the motor speeds        
        if self.speed_ramp < speed_cmd:
            # Speeding up forward or slowing in reverse
            self.speed_ramp = min((self.speed_ramp + ramp_rate), speed_cmd) 
            #self.update_ankle_servos(self.speed_ramp)
            
        elif self.speed_ramp > speed_cmd:
            self.speed_ramp = max((self.speed_ramp - ramp_rate), speed_cmd) 
            #self.update_ankle_servos(self.speed_ramp)

        if False:   # self.speed_ramp != 0.0:
            print("DBG: RAMP: joy=% 2.2f  ramp=% 2.2f  imu=% 2.2f" % 
            (self.speed_cmd, self.speed_ramp, self.imu_pitch_compensated))

        ## Reset collision handling when the robot stops
        ##if self.speed_ramp == 0.0:
            ##self.handling_collision = False # done handling collision

            
        # Return the ramped motor command
        return self.speed_ramp 
        
    def wheel_motor_turn_ramp(self):
        # called each time through the fast loop. Turn command is -1.0 to +1.0
        ramp_seconds = 1.5 # time for full ramp from stop to max turn (shorter than speed_ramp time)
        ramp_rate = 0.02 / ramp_seconds # 0.02 How much to step each time through the 20ms loop (50 times / second)
        turn_cmd =  self.turn_cmd      
        
        # Ramp the motor turns        
        if self.turn_ramp < turn_cmd:
            self.turn_ramp = min((self.turn_ramp + ramp_rate), turn_cmd) 
        elif self.turn_ramp > turn_cmd:
            self.turn_ramp = max((self.turn_ramp - ramp_rate), turn_cmd) 

        if False:   # self.turn_ramp != 0.0:
            print("DBG: RAMP: joy=% 2.2f  ramp=% 2.2f  imu=% 2.2f" % 
            (self.turn_cmd, self.turn_ramp, self.imu_pitch_compensated))
            
        # Return the ramped motor command
        return self.turn_ramp 


    
        
    # ================================================================================================
    # MAIN LOOP
    
    def run(self):
        # Loop 20 ms to process events - 
        # fast servos run at ~16.6 ms, so 20ms loop possible (see Servo README)
        looprate = rospy.Rate(50) # Loops per second (20ms loop)
        while not rospy.is_shutdown(): 
        
            ###print("Wheel Control: Timer Loop!")
            ###print("current time is: ", rospy.get_time())
            ##self.currentMillis = rospy.get_time() * 1000.0 # convert float seconds to ms

            if self.command_timeout_start != None:
                elapsed_time = time.time() - self.command_timeout_start 
                if elapsed_time > COMMAND_TIMEOUT_SECONDS:
                    # Timer has expired without a new command. Stop the wheels.
                    # Programmed move will override this, that's ok.
                    # rospy.loginfo("*********************************************")
                    # rospy.loginfo(" Joystick or command timeout.")
                    # rospy.loginfo(" Stopping unless overridden by programmed move")
                    # rospy.loginfo("*********************************************")
                    self.command_timeout_start = None
                    self.speed_cmd = 0.0
                    self.turn_cmd = 0.0

            # Handle programmed moves
            (update, program_speed, program_turn) = self.programmed_move.update()
            if update and not self.joystick_in_use:
                self.speed_cmd = program_speed
                self.turn_cmd = program_turn

            # check if we need to stop to avoid collision
            self.speed_cmd = self.collision_override(self.speed_cmd)
            
            # Calculate motor speeds, scaling to max Dynamixel MX64 speed
            ramped_speed_command = self.wheel_motor_speed_ramp()            
            forward_speed = ramped_speed_command * DYNAMIXEL_64_MAX_SPEED

            ramped_turn_command = self.wheel_motor_turn_ramp()            
            turn_speed = ramped_turn_command * DYNAMIXEL_64_MAX_SPEED * 0.25 # don't make turns too sensitive
            
            right_speed = self.clamp((forward_speed + turn_speed), DYNAMIXEL_64_MAX_SPEED)  
            left_speed = self.clamp((forward_speed - turn_speed), DYNAMIXEL_64_MAX_SPEED)

            #if  left_speed != 0.0 and right_speed != 0.0:           
                #print("DBG Motors: L = %d, R = %d" % (left_speed, right_speed))

            if self.speed_cmd != 0 or self.speed_ramp != 0 or self.turn_cmd != 0 or self.turn_ramp != 0:
                self.pub_wheel_cmd_right.publish(right_speed) # reversed motor
                self.pub_wheel_cmd_left.publish(-left_speed) 
                #print("Wheel Torque on")
                self.wheel_torque_is_on = True

            elif self.wheel_torque_is_on:           
                # Wheels idle, turn off torque to allow wheels to move freely
                # Allows robot to be pushed, and also keeps servos from heating
                SetServoTorque(0.0, wheel_joints) # 1.0 = max torque
                self.wheel_torque_is_on = False
        
            looprate.sleep()  
 
 
           
        
if __name__ == '__main__':

    # capture SIGINT signal, e.g., Ctrl+C
    # signal.signal(signal.SIGINT, signal_handler)

    node = WheelControl()
    node.run()
    
    # Alternative is to use a timer for the main loop function:
    ## rospy.Timer(rospy.Duration(0.02), node.timer_loop) # 0.01 = 10ms, 0.02 = 20ms




