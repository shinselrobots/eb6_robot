#!/usr/bin/env python3
# Handle buttons on eb to allow servos to be positioned
# Implementation note:  
#   This could (should?) be done with arrays instead of individual variables for each servo,
#   but I wanted to be sure I was getting the right servos in each position (for now)


import rospy
import logging
#import time
#import csv
#import math
#import sys

from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
#from behavior_msgs.msg import CommandState


#from dynamixel_controllers.srv import TorqueEnable, SetServoTorqueLimit, SetSpeed
from eb_servos.set_pose import * # for RobotPose
from eb_servos.servo_joint_list import all_servo_joints, head_joints, right_leg_joints, left_leg_joints
from eb_servos.head_servo_publishers import *
from eb_servos.leg_servo_publishers import *

# from eb_servos.standard_servo_positions import *
from eb_servos.set_servo_speed import *
from eb_servos.set_servo_torque import *
from eb_servos.srv import ReturnJointStates

class PoseButtons():
    def __init__(self):
        rospy.init_node('eb_pose_buttons')

        self.joint_state = JointState()
        self.button_pressed = 0xFF # no button pressed
        self.robotpose = RobotPose("PoseButtons")
        self.saved_servo_position = 0.0
        self.servo_torque_enabled_right = True
        self.servo_torque_enabled_left = True
        self.servo_torque_enabled_center = True
        self.servo_torque_level = 0.8
        self.servo_number = 1  # default to first servo
        self.move_servo_pressed = False
        self.joy_amount_x = 0.0


        # subscribe to servo position messages
        # TODO REMOVE THIS?  >>>>   
        servo_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_cb)

        # subscribe to button messages
        joystick_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        #button_right_sub = rospy.Subscriber('/button_right', Bool, self.button_right_cb)
        #button_left_sub = rospy.Subscriber('/button_left', Bool, self.button_left_cb)
        
        # Publisher for Pose messages (just used to reset to known pose state)
        #self.pub_behavior_pose = rospy.Publisher('behavior/cmd', CommandState, queue_size=2)

        
        # Head Joints
        self.right_antenna = 0.0
        self.last_right_antenna = 0.0
        self.left_antenna = 0.0
        self.last_left_antenna = 0.0
        self.head_sidetilt = 0.0
        self.last_head_sidetilt = 0.0
        self.head_pan = 0.0
        self.last_head_pan = 0.0
        self.head_tilt = 0.0
        self.last_head_tilt = 0.0
        self.neck_raise = 0.0
        self.last_neck_raise = 0.0

        # Hip
        self.leg_hip_lean = 0.0
        self.last_leg_hip_lean = 0.0

        # Right Leg Joints       
        self.right_leg_hip_rotate = 0.0
        self.last_right_leg_hip_rotate = 0.0
        
        self.right_leg_thigh_lift = 0.0
        self.last_right_leg_thigh_lift = 0.0
        
        self.right_leg_knee_bend = 0.0
        self.last_right_leg_knee_bend = 0.0
        
        self.right_leg_ankle_rotate = 0.0
        self.last_right_leg_ankle_rotate = 0.0
        

        # Left Leg Joints
        self.left_leg_hip_rotate = 0.0
        self.last_left_leg_hip_rotate = 0.0
        
        self.left_leg_thigh_lift = 0.0
        self.last_left_leg_thigh_lift = 0.0
        
        self.left_leg_knee_bend = 0.0
        self.last_left_leg_knee_bend = 0.0
        
        self.left_leg_ankle_rotate = 0.0
        self.last_left_leg_ankle_rotate = 0.0
        
        SetServoTorque(self.servo_torque_level, all_servo_joints)
        rospy.loginfo("Pose_Training Ready. Torque applied to all servos.")

        print("")
        print("JoyStick Buttons:")
        print("   A = Reset to default Pose")
        print("   B = Right Leg Torque")
        print("   X = Left Leg Torque")
        print("   Y = Head Torque")
        print("")
        print("   Bottom Right Trigger = Select Servo Increase")
        print("   Bottom Left Trigger =  Select Servo Decrease")
        print("   Top Right Trigger =    Move Servo by Joystick")
        print("   Top Left Trigger =     Reset current servo to last saved position")
        print("")
        print("   Back Button = TBD 8")
        print("   Start Button = TBD 9")
        print("")         
        
        self.saved_servo_position = self.get_servo_position(self.servo_name())
        print("Current Servo = %d, %s, Position = %2.3f" % (self.servo_number, self.servo_name(), self.saved_servo_position))




    def servo_name(self):
        return all_servo_joints[self.servo_number-first_servo_number] # list is zero offset

    def call_return_joint_states(self, joint_names):
        rospy.wait_for_service("return_joint_states")
        try:
            s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
            resp = s(joint_names)
        except rospy.ServiceException as e:
            rospy.logwarn("%s: Error when calling return_joint_states: %s" % ("eb_pose_training", e))

        #print("************* RESULTS *****************")


        for (ind, joint_name) in enumerate(joint_names):
            if(not resp.found[ind]):
                rospy.logwarn("%s: joint %s not found!" % ("eb_pose_training", joint_name ))
            #else:
            #    print("Joint found: %s, Position = %2.3f" % (joint_name, resp.position[ind]))

        return (resp.position, resp.velocity, resp.effort)

    def get_servo_position(self, servo_name):
        # Single servo name, but service expects an array of names
        servo_names = [servo_name]
        # print(servo_names)        
        (position, velocity, effort) = self.call_return_joint_states(servo_names)
        servo_pos = position[0]
        return servo_pos



    def servo_timer(self, event=None):
        # print("servo_timer called!")
        if self.move_servo_pressed:
        
            servo_name = self.servo_name()
            #print("servo name = %s" % servo_name)
            #servo_names = [servo_name]
            #print(servo_names)
            
            #self.call_return_joint_states(servo_names) # service expects an array of names
   
            servo_position = self.get_servo_position(servo_name)         
            #print("*********** SERVO Position = %2.3f" % servo_position)
            new_servo_position = servo_position +  (self.joy_amount_x * .2)      
            
            #print("Moving Servo = %d, %s, New Position = %2.3f" % (self.servo_number, self.servo_name(), new_servo_position))

            self.publish_new_position(servo_name, new_servo_position)



    # ------------------- JOYSTICK CALLBACK ---------------------------
    def joy_callback(self, data):
        # Joystick messages
        # Only one button supported at a time

        self.button_pressed = -1
        for i in range(0,12): 
            #print(i)               
            if data.buttons[i] == 1:  # Button Pressed
                self.button_pressed = i
                break

        # print("Button Pressed = ", self.button_pressed)

        if self.button_pressed == -1: # No button pressed
        
            if self.move_servo_pressed:
                print("DBG - move_servo_pressed")
                
                # Move servo was pressed. Print servo position on the button up.
                servo_position = self.get_servo_position(self.servo_name())
                print("Servo %d, %s, New Position = %2.3f" % (self.servo_number, self.servo_name(), servo_position))
                self.move_servo_pressed = False
            
            if not self.servo_torque_enabled_left:
                # re-enable servo torque on button up
                SetServoTorque(self.servo_torque_level, left_leg_joints) # restore torque
                self.print_leg_servos_left() # print servo values
                self.servo_torque_enabled_left = True
                
            if not self.servo_torque_enabled_right:
                # re-enable servo torque on button up
                SetServoTorque(self.servo_torque_level, right_leg_joints) # restore torque
                self.print_leg_servos_right() # print servo values
                self.servo_torque_enabled_right = True

            if not self.servo_torque_enabled_center:
                # re-enable servo torque on button up
                SetServoTorque(self.servo_torque_level, head_joints) # restore torque
                self.print_head_servos() # print servo values
                self.servo_torque_enabled_center = True

                
        elif self.button_pressed == 0: # Left (X button)
            rospy.loginfo("left button down. Removing leg torque!")
            SetServoTorque(0.0, left_leg_joints) # remove torque
            self.servo_torque_enabled_left = False


        elif self.button_pressed == 3: # Head (Y button)
            rospy.loginfo("Top button down. Removing head torque!")
            SetServoTorque(0.0, head_joints) # remove torque
            self.servo_torque_enabled_center = False

        elif self.button_pressed == 2: # Right (B button)
            rospy.loginfo("right button down. Removing leg torque!")
            SetServoTorque(0.0, right_leg_joints) # remove torque
            self.servo_torque_enabled_right = False

        elif self.button_pressed == 1: # Reset Pose (A button)
            rospy.loginfo("Resetting Pose")
            goal_pose = 0 # 0 = sleep position, 1 = Sitting with head up
            slowest_servo_speed = 0.3
            self.robotpose.move(goal_pose, slowest_servo_speed)


        # Do servo movements

        elif self.button_pressed == 6: # Servo Number Decrease (Left bottom trigger)
            self.servo_number = self.servo_number - 1
            if self.servo_number < first_servo_number:  # defined in servo_joint_list.py
                self.servo_number = first_servo_number
                print("**** FIRST SERVO ****")
                
            servo_position = self.get_servo_position(self.servo_name())         
            print("New Servo = %2d %36s   Position = %2.3f" % (self.servo_number, self.servo_name(), servo_position))
            self.saved_servo_position = servo_position


        elif self.button_pressed == 7: # Servo Number Increase (Right bottom trigger)
            self.servo_number = self.servo_number + 1
            if self.servo_number > last_servo_number:  # defined in servo_joint_list.py
                self.servo_number = last_servo_number
                print("**** LAST SERVO ****")
                
            servo_position = self.get_servo_position(self.servo_name())         
            print("New Servo = %2d %36s   Position = %2.3f" % (self.servo_number, self.servo_name(), servo_position))
            self.saved_servo_position = servo_position


        elif self.button_pressed == 5: # Servo Positon change (Right top trigger)
            self.move_servo_pressed = True # Enable timer to move servos



        elif self.button_pressed == 4: # Servo Positon Reset (Left top trigger)
            # reset to position servo was in when selected
            print("RESTORING SERVO TO PRIOR POSITION %2.3f" % self.saved_servo_position)
            self.publish_new_position(self.servo_name(), self.saved_servo_position)

        
        else:
            print("Unhandled button (%d) Ignored." % self.button_pressed)
            
        if self.move_servo_pressed:
            # Stays true until button released            
            self.joy_amount_x = data.axes[2 ] # Joystick X input (+1.0 to -1.0)


    def publish_new_position(self, servo_name, position):

        if servo_name == 'right_antenna_joint':
            pub_right_antenna.publish(position)
        elif servo_name == 'left_antenna_joint':
            pub_left_antenna.publish(position)
            
        elif servo_name == 'head_sidetilt_joint':
            pub_head_sidetilt.publish(position)
        elif servo_name == 'head_pan_joint':
            pub_head_pan.publish(position)
        elif servo_name == 'head_tilt_joint':
            pub_head_tilt.publish(position)
            
        elif servo_name == 'neck_raise_joint':
            pub_neck_raise.publish(position)
            
        #elif servo_name == 'leg_hip_lean_joints':
        #    pub_leg_hip_lean.publish(position)
            
        elif servo_name == 'right_leg_hip_rotate_joint':
            pub_right_leg_hip_rotate.publish(position)
        elif servo_name == 'right_leg_thigh_lift_joint':
            pub_right_leg_thigh_lift.publish(position)
        elif servo_name == 'right_leg_knee_bend_joint':
            pub_right_leg_knee_bend.publish(position)
        elif servo_name == 'right_leg_ankle_rotate_joint':
            pub_right_leg_ankle_rotate.publish(position)
            
        elif servo_name == 'left_leg_hip_rotate_joint':
            pub_left_leg_hip_rotate.publish(position)
        elif servo_name == 'left_leg_thigh_lift_joint':
            pub_left_leg_thigh_lift.publish(position)
        elif servo_name == 'left_leg_knee_bend_joint':
            pub_left_leg_knee_bend.publish(position)
        elif servo_name == 'left_leg_ankle_rotate_joint':
            pub_left_leg_ankle_rotate.publish(position)
            
        else:
            print("ERROR: Unknown Servo. Ignored.")
    

    
    # BUTTON EVENTS.  
    # Button Down used to relax servos for positioning, Button Up to log servo positions
    def button_right_cb(self, msg):
        button_pressed = msg.data

        #rospy.loginfo("DEBUG got right button event.")
        if button_pressed:
            rospy.loginfo("right button down event. Removing leg torque!")
            SetServoTorque(0.0, right_leg_joints) # remove torque

        else:
            rospy.loginfo("right button up event. Logging servo positions")
            self.marker_flag_set = True    # Force current servo positions to be written
            SetServoTorque(0.5, right_leg_joints) # restore torque

            # PRINT SERVO VALUES
            self.print_leg_servos_right()

    def button_left_cb(self, msg):
        button_pressed = msg.data

        #rospy.loginfo("DEBUG got left button event.")
        if button_pressed:
            rospy.loginfo("left button down event. Removing leg torque!")
            SetServoTorque(0.0, left_leg_joints) # remove torque

        else:
            rospy.loginfo("left button up event. Logging servo positions")
            self.marker_flag_set = True    # Force current servo positions to be written
            SetServoTorque(0.5, left_leg_joints) # restore torque

            # PRINT SERVO VALUES
            self.print_leg_servos_left()




    def joint_state_cb(self, msg):
        #rospy.loginfo("joint_state_cb called")


        # joint_state messages can have any number of servos, 
        # so need to check to see which ones are included in each message 
        try:
            self.right_antenna = msg.position[
              msg.name.index('right_antenna_joint')]
            #rospy.loginfo( self.right_antenna_joint + " = " + str(self.right_antenna))
        except Exception:
            pass # ignore exception
        try:
            self.left_antenna = msg.position[
              msg.name.index('left_antenna_joint')]
            #rospy.loginfo( self.left_antenna_joint + " = " + str(self.left_antenna))
        except Exception:
            pass # ignore exception
        try:
            self.head_pan = msg.position[
              msg.name.index('head_pan_joint')]
            #rospy.loginfo( self.head_pan_joint + " = " + str(self.head_pan))
        except Exception:
            pass # ignore exception
        try:
            self.head_tilt = msg.position[
              msg.name.index('head_tilt_joint')]
        except Exception:
            pass # ignore exception
        try:
            self.head_sidetilt = msg.position[
              msg.name.index('head_sidetilt_joint')]
        except Exception:
            pass # ignore exception
        try:
            self.neck_raise = msg.position[
              msg.name.index('neck_raise_joint')]
        except Exception:
            pass # ignore exception


        #try:
        #    self.leg_hip_lean = msg.position[
        #      msg.name.index('leg_hip_lean_joints')]
        #except Exception:
        #    pass # ignore exception


        try:
            self.right_leg_hip_rotate = msg.position[
              msg.name.index('right_leg_hip_rotate_joint')]
        except Exception:
            pass # ignore exception
        try:
            self.right_leg_thigh_lift = msg.position[
              msg.name.index('right_leg_thigh_lift_joint')]
        except Exception:
            pass # ignore exception
        try:
            self.right_leg_knee_bend = msg.position[
              msg.name.index('right_leg_knee_bend_joint')]
        except Exception:
            pass # ignore exception
        try:
            self.right_leg_ankle_rotate = msg.position[
              msg.name.index('right_leg_ankle_rotate_joint')]
        except Exception:
            pass # ignore exception


        try:
            self.left_leg_hip_rotate = msg.position[
              msg.name.index('left_leg_hip_rotate_joint')]
        except Exception:
            pass # ignore exception
        try:
            self.left_leg_thigh_lift = msg.position[
              msg.name.index('left_leg_thigh_lift_joint')]
        except Exception:
            pass # ignore exception
        try:
            self.left_leg_knee_bend = msg.position[
              msg.name.index('left_leg_knee_bend_joint')]
        except Exception:
            pass # ignore exception
        try:
            self.left_leg_ankle_rotate = msg.position[
              msg.name.index('left_leg_ankle_rotate_joint')]
        except Exception:
            pass # ignore exception

    def print_leg_servos_right(self):

        rospy.loginfo("-----------------------------------")
        rospy.loginfo("leg_hip_lean           = %7.4f", self.leg_hip_lean)
        rospy.loginfo("right_leg_hip_rotate   = %7.4f", self.right_leg_hip_rotate)
        rospy.loginfo("right_leg_thigh_lift   = %7.4f", self.right_leg_thigh_lift)
        rospy.loginfo("right_leg_knee_bend    = %7.4f", self.right_leg_knee_bend)
        rospy.loginfo("right_leg_ankle_rotate = %7.4f", self.right_leg_ankle_rotate)
        rospy.loginfo("-----------------------------------")

    def print_leg_servos_left(self):
        rospy.loginfo("-----------------------------------")
        rospy.loginfo("leg_hip_lean           = %7.4f", self.leg_hip_lean)
        rospy.loginfo("left_leg_hip_rotate    = %7.4f", self.left_leg_hip_rotate)
        rospy.loginfo("left_leg_thigh_lift    = %7.4f", self.left_leg_thigh_lift)
        rospy.loginfo("left_leg_knee_bend     = %7.4f", self.left_leg_knee_bend)
        rospy.loginfo("left_leg_ankle_rotate  = %7.4f", self.left_leg_ankle_rotate)
        rospy.loginfo("-----------------------------------")

    def print_head_servos(self):
        rospy.loginfo("-----------------------------------")
        rospy.loginfo("right_antenna          = %7.4f", self.right_antenna)
        rospy.loginfo("left_antenna           = %7.4f", self.left_antenna)
        rospy.loginfo("head_sidetilt          = %7.4f", self.head_sidetilt)
        rospy.loginfo("head_pan               = %7.4f", self.head_pan)
        rospy.loginfo("head_tilt              = %7.4f", self.head_tilt)
        rospy.loginfo("neck_raise             = %7.4f", self.neck_raise)
        rospy.loginfo("-----------------------------------")

        
if __name__=='__main__':

        
        pose_buttons = PoseButtons()
        rospy.Timer(rospy.Duration(1.0/10.0), pose_buttons.servo_timer)
        rospy.spin()



