#!/usr/bin/env python3
# Record servo positions for later scripting of robot behavior
# Implementation note:  
#   This could (should?) be done with arrays instead of individual variables for each servo,
#   but I wanted to be sure I was getting the right servos in each position (for now)
# If a parser is used, it should ignore any lines that start with '#'


# IMPORTANT WARNING - For Sheldon, need to update Sheldon specific version of this file!

import rospy
import logging
import time
import csv
import math
import sys

from sensor_msgs.msg import Joy
#from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import JointStateArray

from std_msgs.msg import UInt16
from std_msgs.msg import Bool


# EB Robot ONLY
#from dynamixel_controllers.srv import TorqueEnable, SetServoTorqueLimit, SetSpeed
from eb_servos.set_pose import *    # for RobotPose
from eb_servos.servo_joint_list import all_servo_joints, head_joints, right_leg_joints, left_leg_joints
from eb_servos.head_servo_publishers import *
from eb_servos.leg_servo_publishers import *

#from eb_servos.standard_servo_positions import *
from eb_servos.set_servo_speed import *
from eb_servos.set_servo_torque import *
from eb_servos.srv import ReturnJointStates

# Tune this to make movements not jerkey by too many intermediate positions
# Head probably likes smaller numbers, legs bigger numbers...
MOVEMENT_THRESHOLD = 0.0435 # 2.5 degrees
#MOVEMENT_THRESHOLD = 0.087 # 5 degrees
#MOVEMENT_THRESHOLD = 0.175 # 10 degrees
#MOVEMENT_THRESHOLD = 0.349 # 20 degrees

FIXED_STEP_SECONDS = 0.960
DEFAULT_POSE = 4

class RecordServoPositions():
    def __init__(self):
        rospy.init_node('record_servos')
        print("*************** Starting Record Servos **************")

        self.joint_state = JointStateArray()
        self.robotpose = RobotPose("record_servo")
        self.record_mode = 'all' # 'changed' = servos that moved > threshold
        self.step_number = 0
        self.marker_index = 0

        self.button_pressed = 0xFF # no button pressed
        self.saved_servo_position = 0.0
        self.servo_torque_enabled_right = True
        self.servo_torque_enabled_left = True
        self.servo_torque_enabled_head = True
        self.servo_torque_enabled_current = True
        self.servo_torque_level = 0.8
        self.servo_number = 1;  # default to first servo
        self.move_servo_pressed = False
        self.joy_amount_x = 0.0
        self.set_pose_topic = '/set_pose'

        # subscribe to servo position messages
        # servo_sub = rospy.Subscriber('/joint_states_array', JointStateArray, self.joint_state_cb)

        # subscribe to marker messages by user, to indicate events in the time stream (such as "servos in position #4")
        time_mark_sub = rospy.Subscriber('/user_mark_time', UInt16, self.time_mark_cb) # arbitrary ID assigned by user
        
        # subscribe to pose command messages, to allow setting default pose from another shell
        set_pose_sub = rospy.Subscriber(self.set_pose_topic, UInt16, self.set_pose_cb)
        

        # use joystick buttons to trigger recording
        joystick_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)


        # Use Python's CSV writer
        self.csv_file_name = "/home/system/record_servos.csv"
        self.csv_file = open(self.csv_file_name, "w") # use "a" to append
        fieldnames = ['comment', 'step', 'time', 'type', 'head_pan', 'head_tilt', 'head_sidetilt', 'neck_raise', 'right_antenna', 'left_antenna', 'leg_hip_lean',
            'r_leg', 'right_leg_hip_rotate', 'right_leg_thigh_lift', 'right_leg_knee_bend', 'right_leg_ankle_rotate',
            'l_leg', 'left_leg_hip_rotate', 'left_leg_thigh_lift', 'left_leg_knee_bend', 'left_leg_ankle_rotate',
            'param1', 'param2']

        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames)
        self.csv_writer.writeheader()
        self.start_time = rospy.get_rostime()
        self.last_elapsed_sec = 0.0

        # Save servo position commands as arrays in ID order
        self.servo_names = all_servo_joints
        self.last_cmd = []
        # self.current_cmd = []
        # initialize the arrays       
        for i in range(len(self.servo_names)):
            # print( str(i) + " :  [" + self.servo_names[i] + "]" )
            self.last_cmd.append(0.0)
            # self.current_cmd.append(0.0)


        
        
        SetServoTorque(self.servo_torque_level, all_servo_joints)

        print("")
        print("JoyStick Buttons:")
        print("   A = Reset to default Pose. To change: \'rostopic pub -1 %s std_msgs/UInt16 <pose_num>\'" % self.set_pose_topic)
        print("   B = Right Leg Torque")
        print("   X = Left Leg Torque")
        print("   Y = Head Torque")
        print("")
        print("   Bottom Right Trigger = Select Servo Increase")
        print("   Bottom Left Trigger =  Select Servo Decrease")
        print("   Top Right Trigger =    Move Servo by Joystick")
        print("   Top Left Trigger =     Reset current servo to last saved position")
        print("")
        print("   Back Button = Record and Mark Left")
        print("   Start Button = Record and Mark Right")
        print("")         
        print("Record Servos Ready. Torque applied to all servos.")
        print("Recording to file %s", self.csv_file_name)
        print("")         
        self.saved_servo_position = self.get_servo_position(self.servo_name())
        print("Current Servo = %d, %s, Position = %2.3f" % (self.servo_number, self.servo_name(), self.saved_servo_position))
        
        print("waiting for return_joint_states service...")
        rospy.wait_for_service("return_joint_states")
        print("service found.")
        print("Record Servos ready.")
        

    def __del__(self):
        self.csv_file.close()
        print("Record to file %s - file closed.", self.csv_file_name)


    def time_mark_cb(self, msg):
        rospy.loginfo("got time ID marker from user")

        id = msg.data
        rospy.loginfo("================ GOT MARKER! ================") 
        self.csv_writer.writerow({
            'comment': ' ',
            'step': '0 ',
            'time': '0.0  ',
            'type': 'marker',
            'param1': '{:d}'.format(id)})

    def set_pose_cb(self, msg):
        rospy.loginfo("got new pose from user")

        # print("DBG: Message.data = ", msg.data)
        #rx_data = msg.data
        
        #print("rx = ", rx_data)
        
        self.goal_pose = msg.data
        
        rospy.loginfo("----------> New Pose set to: %d" % self.goal_pose) # 0 = sleep position, 1 = Sitting with head up, etc.
        slowest_servo_speed = 0.3
        self.robotpose.move(self.goal_pose, slowest_servo_speed)



    def write_servos_to_csv(self):
        # Write position of servos.
        # if record_mode == all, record all servo positions
        # if record_mode == changed, record servos that moved > threshold
        #print("DBG: write_servos_to_csv() called.")
        
        # Pass service an array of names, and get all values, zero indexed.
        (position, velocity, effort, goal_pos, error) = self.call_return_joint_states(all_servo_joints)
        index = 0
        csv_log_value = []
        for servo_name in all_servo_joints:
            servo_number = first_servo_number + index      # first_servo_number is defined in servo_joint_list.py   


            report_val = float('nan') # May be actual value or "nan" if not changed
            current_cmd = goal_pos[index]
            print_nan = 'Nan'
            if self.record_mode == 'all': # record all servos cmds, even if they didn't move
                report_val =  current_cmd 
                self.last_cmd[index] = current_cmd
                print_nan = '   '

            elif (math.fabs(current_cmd - self.last_cmd[index]) > MOVEMENT_THRESHOLD):   
                report_val =  current_cmd # servo moved, so record it
                # print("DBG: report_val = %1.4f, current_val = %1.4f, last_val = %1.4f" % (report_val, current_val, last_val ))
                self.last_cmd[index] = current_cmd
                print_nan = '   '

            # if none of the above, the value is reported as NAN, and the last_cmd is unchanged.       
            csv_log_value.append(report_val)

            # Display for the user
            print("Servo %2d %36s  %s Cmd = % 2.3f,  Position = % 2.3f,  Error = % 2.3f,  Effort = % 2.3f" %
                (servo_number, servo_name, print_nan, goal_pos[index], position[index], error[index], effort[index]))

            if servo_number == 6 or servo_number == 11:
                print("-----------------------------------")

            index = index + 1
        print("-----------------------------------")



        # row types:  move, speed (to set servo speeds), speak, lights, etc.
        row_type = 'move'

        rospy.loginfo("DBG: ========================= WRITING! ======================")

        elapsed_time = rospy.Time.now() - self.start_time
        elapsed_sec = elapsed_time.to_sec()
        step_seconds = elapsed_sec - self.last_elapsed_sec # log the time between steps
        rospy.loginfo("DBG: step_seconds = %f, elapsed_sec = %f, last_elapsed_sec = %f", 
            step_seconds, elapsed_sec, self.last_elapsed_sec)
        self.last_elapsed_sec = elapsed_sec

        self.step_number += 1

        # Not that the columns are named (see fieldnames), so order of writing is not important
        self.csv_writer.writerow({
            'comment': ' ',
            'step': '{:d}'.format(self.step_number),
            'time': '{:1.3f}'.format(step_seconds),
            'type': row_type,
            
            'right_antenna': '{:1.4f}'.format(csv_log_value[0]), 
            'left_antenna': '{:1.4f}'.format(csv_log_value[1]),
            
            'head_pan': '{:1.4f}'.format(csv_log_value[2]), 
            'head_tilt': '{:1.4f}'.format(csv_log_value[3]),
            'head_sidetilt': '{:1.4f}'.format(csv_log_value[4]),
            'neck_raise': '{:1.4f}'.format(csv_log_value[5]),

            'leg_hip_lean': '{:1.4f}'.format(csv_log_value[6]),

            'l_leg': 'L:',      # make file easier for humans to read
            'left_leg_hip_rotate': '{:1.4f}'.format(csv_log_value[7]),
            'left_leg_thigh_lift': '{:1.4f}'.format(csv_log_value[8]), 
            'left_leg_knee_bend': '{:1.4f}'.format(csv_log_value[9]),
            'left_leg_ankle_rotate': '{:1.4f}'.format(csv_log_value[10]), 

            'r_leg': 'R:',      # make file easier for humans to read
            'right_leg_hip_rotate': '{:1.4f}'.format(csv_log_value[11]), 
            'right_leg_thigh_lift': '{:1.4f}'.format(csv_log_value[12]),
            'right_leg_knee_bend': '{:1.4f}'.format(csv_log_value[13]),
            'right_leg_ankle_rotate': '{:1.4f}'.format(csv_log_value[14]),


            # user will edit CSV to add parameters for things like text to speak, light state, etc.
            'param1': "",  
            'param2': ""
            })



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
            
        elif servo_name == 'leg_hip_lean_joint':
            pub_leg_hip_lean.publish(position)
            
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
    
    def joystick_servo_timer(self, event=None):
        # used to make incremental changes. More joystick = faster changes
        #print("joystick_servo_timer called!")
        if self.move_servo_pressed:
        
            servo_name = self.servo_name()
            #print("servo name = %s" % servo_name)
            #servo_names = [servo_name]
            #print(servo_names)
            
            #self.call_return_joint_states(servo_names) # service expects an array of names
   
            position = self.get_servo_position(servo_name)         
            #print("*********** SERVO Position = %2.3f" % position)
            new_servo_position = position +  (self.joy_amount_x * .2)      
            
            #print("Moving Servo = %d, %s, New Position = %2.3f" % (self.servo_number, self.servo_name(), new_servo_position))

            self.publish_new_position(servo_name, new_servo_position)


        
    # =============================== JOYSTICK CALLBACK =================================
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

                #position = self.get_servo_position(self.servo_name())
                #print("Servo %d, %s, New Position = %2.3f" % (self.servo_number, self.servo_name(), position))

                (position, velocity, effort, goal_pos, error) = self.get_servo_full_status(self.servo_name())
                print("Servo %2d %36s   Position = %2.3f, Cmd = %2.3f, Error = %2.3f, Effort = %2.3f" %
                    (self.servo_number, self.servo_name(), position, goal_pos, error, effort))

                self.move_servo_pressed = False
                           
           
        # Large servo movements (manually move joints)
        # Toggle servo torque on/off. Button orientation for robot facing user
            
        elif self.button_pressed == 0: # right (X button)
            if self.servo_torque_enabled_right:
                rospy.loginfo("Removing right leg torque!")
                SetServoTorque(0.0, right_leg_joints) # remove torque
                self.servo_torque_enabled_right = False
            else:
                rospy.loginfo("Applying right leg torque!")
                SetServoTorque(self.servo_torque_level, right_leg_joints) # restore torque
                self.servo_torque_enabled_right = True
            
        elif self.button_pressed == 2: # Left (B button)
            if self.servo_torque_enabled_left:
                rospy.loginfo("Removing left leg torque!")
                SetServoTorque(0.0, left_leg_joints) # remove torque
                self.servo_torque_enabled_left = False
            else:
                rospy.loginfo("Applying left leg torque!")
                SetServoTorque(self.servo_torque_level, left_leg_joints) # restore torque
                self.servo_torque_enabled_left = True

        elif self.button_pressed == 3:  # Head (Y button)
            print("Y Button: Next Step (handled by playback_servos)")
            
        elif self.button_pressed == 99:  # 3 DISABLED!!! Head (Y button)
            if self.servo_torque_enabled_head:
                rospy.loginfo("Removing head torque!")
                SetServoTorque(0.0, head_joints) # remove torque
                self.servo_torque_enabled_head = False
            else:
                rospy.loginfo("Applying head torque!")
                SetServoTorque(self.servo_torque_level, head_joints) # restore torque
                self.servo_torque_enabled_head = True

        elif self.button_pressed == 1: # Reset Pose (A button)
            # TODO
            rospy.loginfo("Resetting Pose - TODO")
            goal_pose = DEFAULT_POSE # 0 = sleep position, 1 = Sitting with head up
            slowest_servo_speed = 0.3
            self.robotpose.move(goal_pose, slowest_servo_speed)


        # Small servo movements

        elif self.button_pressed == 6: # Servo Number Decrease (Left bottom trigger)
            self.servo_number = self.servo_number - 1
            if self.servo_number < first_servo_number:  # defined in servo_joint_list.py
                self.servo_number = first_servo_number
                print("**** FIRST SERVO ****")
                
            #position = self.get_servo_position(self.servo_name())         
            #print("New Servo = %2d %36s   Position = %2.3f" % (self.servo_number, self.servo_name(), position))

            (position, velocity, effort, goal_pos, error) = self.get_servo_full_status(self.servo_name())
            print("Servo %2d %36s   Position = %2.3f, Cmd = %2.3f, Error = %2.3f, Effort = %2.3f" %
                (self.servo_number, self.servo_name(), position, goal_pos, error, effort))
            self.saved_servo_position = position


        elif self.button_pressed == 7: # Servo Number Increase (Right bottom trigger)
            self.servo_number = self.servo_number + 1
            if self.servo_number > last_servo_number:  # defined in servo_joint_list.py
                self.servo_number = last_servo_number
                print("**** LAST SERVO ****")
                
            #position = self.get_servo_position(self.servo_name())         
            #print("New Servo = %2d %30s   Position = %2.3f" % (self.servo_number, self.servo_name(), position))
            (position, velocity, effort, goal_pos, error) = self.get_servo_full_status(self.servo_name())
            print("Servo %2d %36s   Position = %2.3f, Cmd = %2.3f, Error = %2.3f, Effort = %2.3f" %
                (self.servo_number, self.servo_name(), position, goal_pos, error, effort))
            self.saved_servo_position = position


        elif self.button_pressed == 5: # Servo Positon change (Right top trigger)
            self.move_servo_pressed = True # Enable timer to move servos


        #elif self.button_pressed == 4: # Servo Positon Reset (Left top trigger)
        #    # reset to position servo was in when selected
        #    print("RESTORING SERVO TO PRIOR POSITION %2.3f" % self.saved_servo_position)
        #    self.publish_new_position(self.servo_name(), self.saved_servo_position)

        elif self.button_pressed == 4: # Servo Torque On/Off (Left top trigger)
            if self.servo_torque_enabled_current:
                rospy.loginfo("Removing current servo torque!")
                SetSingleServoTorque(0.0, self.servo_name()) # remove torque
                self.servo_torque_enabled_current = False
            else:
                rospy.loginfo("Applying current servo torque!")
                SetSingleServoTorque(self.servo_torque_level, self.servo_name()) # restore torque
                self.servo_torque_enabled_current = True

                position = self.get_servo_position(self.servo_name())         
                print("New Servo = %2d %30s   Position = %2.3f" % (self.servo_number, self.servo_name(), position))
                self.saved_servo_position = position
            

        elif self.button_pressed == 8: # Event Left (Back Button)
            rospy.loginfo("Back Button Event. Logging MOVED servo positions")
            self.record_mode = 'changed'    # only record servos that changed 

            self.write_servos_to_csv()



        elif self.button_pressed == 9: # Event Right (Start Button)
            rospy.loginfo("Start Button Event. Logging ALL servo positions")
            self.record_mode = 'all'    # Force all servos to be written

            #self.marker_index += 1
            #rospy.loginfo("================ WRITING MARKER! ================") 
            #self.csv_writer.writerow({
            #    'comment': ' ',
            #    'step': '0 ',
            #    'time': '0.0  ',
            #    'type': 'marker',
            #    'param1': '{:d}'.format(self.marker_index)})

            self.write_servos_to_csv()

        
        else:
            print("Unhandled button. Ignored.")

        if self.move_servo_pressed:
            # Stays true until button released. Update joystick position.            
            self.joy_amount_x = data.axes[2 ] # Joystick X input (+1.0 to -1.0)
        
        
        
if __name__=='__main__':

    record_positions = RecordServoPositions()
    rospy.Timer(rospy.Duration(1.0/10.0), record_positions.joystick_servo_timer)
    rospy.spin()



