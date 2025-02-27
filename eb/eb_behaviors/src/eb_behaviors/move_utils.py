#! /usr/bin/env python
# Utilities to perform wheel movements
# Used by behaviors that need this capability

import rospy
import time
import math
import random
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Empty

# For wheel movement
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Point32, Pose, Quaternion, Twist, Vector3

# ROS Angles package
#includes: fabs, shortest_angular_distance, normalize_angle_positive, etc.
from angles import * # Python dir() to see list

# for servos
#from eb_servos.head_servo_publishers import *
#from eb_servos.right_arm_servo_publishers import *
#from eb_servos.left_arm_servo_publishers import *

from eb_servos.set_pose import *
#from eb_servos.standard_servo_positions import *
#from eb_servos.set_servo_speed import *
#from eb_servos.set_servo_torque import *

from simple_move_msgs.msg import SimpleMove


DEFAULT_MOVE_SPEED = 0.40 
MIN_MOVE_SPEED = 0.20
MOVE_STOP_FUDGE = 0.02 # about 8 inches

DEFAULT_TURN_SPEED = 0.50 # half max
MIN_TURN_SPEED = 0.20
TURN_STOP_FUDGE = 0.080 # radians

class MoveUtils():
    # Move specified amount forward or back
    
    def __init__(self, interrupt_check):
        self.module_name = 'move_utils'
        rospy.loginfo('%s: Initializing...' % (self.module_name))
        self.interrupt_check = interrupt_check

        self.current_odom = 0.0
        self.last_odom = 0.0
        self.goal_move_speed = 0.0
        self.goal_move_distance = 0.0
        self.slow_move_speed = MIN_MOVE_SPEED # slow down at the end for precision
        
        # Publisher for wheel motor commands
        #self.pub_wheel_motors = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_simple_wheel_move = rospy.Publisher('/simple_move', SimpleMove, queue_size=1)

        odom_sub = rospy.Subscriber("/simple_odom", Float64, self.odom_callback)
        
        rospy.loginfo("%s: init complete." % (self.module_name))


    def max_clamp(self, value, max_value): # clamp pos and neg max_values
        return max(min(value, max_value), -max_value)


    def odom_callback(self, data): # Meters
        #print("got odom: ", data) 
        self.current_odom = float(data.data)


    def begin_move(self, move_amount, move_speed=DEFAULT_MOVE_SPEED):
        rospy.loginfo('%s: begin_move' % (self.module_name))
        rospy.loginfo( "  requested_distance: %2.2f" % move_amount) # # distance in meters (positive = forward)
        rospy.loginfo( "  requested_speed:    %2.2f" % move_speed) # move speed from MIN_MOVE_SPEED to 1.0 (always positive)

        self.goal_move_distance = move_amount
        
        # Confirm Odom is reporting position
        while False: # self.current_odom == 0.0:
            rospy.logwarn('%s: No ODOM data received! Waiting for Odom!' % (self.module_name))
            if self.interrupt_check():
                rospy.loginfo('%s: Interrupt Detected. Exiting Behavior.' % self.module_name)
                break
            rospy.sleep(0.1)
                        
        # Get initial position odometry
        self.start_odom = self.current_odom
        rospy.loginfo("%s: Start Position: %2.4f" % (self.module_name, self.start_odom))

        # Start the move - simple move command
        self.pub_simple_wheel_move.publish(move_amount, move_speed, 0.0, 0.0)

        return True # good start       


    def check_move_complete(self):
        move_progress = self.current_odom - self.start_odom
        #print("DBG check_move_complete: move_progress = ", move_progress)
        remaining = self.goal_move_distance - fabs(move_progress)
        rospy.loginfo('%s: Move Remaining: %02.4f' % (self.module_name, remaining))
 
        if remaining <= MOVE_STOP_FUDGE:   
            rospy.loginfo('%s: move Completed.' % self.module_name)
            return True

        return False  # still turning


    def move_cleanup(self):
        # un-mute the microphone
        # self.mic_system_enable_pub.publish(True)
        rospy.loginfo('%s: move complete' % self.module_name)





# -------------------------- TURN UTILS ---------------------------------------
class TurnUtils():

    def __init__(self, interrupt_check):
        self.module_name = 'turn_utils'
        rospy.loginfo('%s: Initializing...' % (self.module_name))
        self.interrupt_check = interrupt_check

        self.imu_yaw = 0.0
        self.last_angle_rad = 0.0
        self.goal_turn_speed = 0.0
        self.slow_turn_speed = MIN_TURN_SPEED # slow down at the end for precision

        # Publisher for wheel motor commands
        self.pub_wheel_motors = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        imu_sub = rospy.Subscriber("/imu_orientation", Point32, self.imu_callback)
        
        rospy.loginfo("%s: init complete." % (self.module_name))


    def imu_callback(self, data): # Degrees
        #print("got imu: ", data) 
        #self.imu_roll = data.x
        #self.imu_pitch = data.y
        self.imu_yaw = data.z


    def begin_turn(self, turn_amount, turn_speed=DEFAULT_TURN_SPEED):
        rospy.loginfo('%s: begin_turn' % (self.module_name))
        rospy.loginfo("  Turn Amount: %2.2f" % turn_amount) # +/- turn amount in degrees. Positive = left
        rospy.loginfo("  Turn Speed:  %2.2f" % turn_speed) # Turn speed from MIN_TURN_SPEED to 1.0 (always positive)

        # Speed
        if turn_speed <  MIN_TURN_SPEED:
            rospy.logwarn('%s: Requested turn speed [%2.2f] < MIN_TURN_SPEED! SPEED SET TO MIN.' % (self.module_name, turn_speed))
            turn_speed = MIN_TURN_SPEED

        # apply turn direction to speed command
        if turn_amount < 0.0:
            self.goal_turn_speed = turn_speed * -1.0
            self.slow_turn_speed = MIN_TURN_SPEED * -1.0
        else:
            self.goal_turn_speed = turn_speed
            self.slow_turn_speed = MIN_TURN_SPEED

        rospy.loginfo('%s:begin_turn: goal speed = %2.2f, min speed = %2.2f' % (self.module_name, self.goal_turn_speed, self.slow_turn_speed))

        # Turn
        abs_turn_amount_deg = fabs(turn_amount) # Turn has no sign
        if abs_turn_amount_deg < 5.0: 
            rospy.logwarn('%s: Requested turn amount [%2.2f] < 5 degrees! Turn Cancelled.' % (self.module_name, abs_turn_amount_deg))
            self.turn_cleanup()
            return False
            
        self.turn_remaining_rad = math.radians(abs_turn_amount_deg)
        rospy.loginfo("%s: Turn amount requested: %f rad, %f deg" % (self.module_name, self.turn_remaining_rad, abs_turn_amount_deg))
             
        # Confirm IMU is reporting position
        while self.imu_yaw == 0.0:
            rospy.logwarn('%s: No IMU data received! Waiting for IMU!' % (self.module_name))
            if self.interrupt_check():
                rospy.loginfo('%s: Interrupt Detected. Exiting Behavior.' % self.module_name)
                break
            rospy.sleep(0.1)
                        
        # Get initial rotation position
        self.last_angle_rad = normalize_angle_positive(math.radians(self.imu_yaw))

        rospy.loginfo("%s: Start Angle: %f radians, %f degrees" % 
            (self.module_name, self.last_angle_rad, math.degrees(self.last_angle_rad)))

        # Start the turn
        self.publish_turn_command(self.goal_turn_speed)
        
        return True # good start 
        

    def check_turn_complete(self):
        current_angle_rad = normalize_angle_positive(math.radians(self.imu_yaw))
        #print("DBG check_turn_complete: current_angle_radians = ", current_angle_rad)

        # handle wrap-around
        turn_progress = fabs(shortest_angular_distance(self.last_angle_rad, current_angle_rad));
        if turn_progress < 0.0: # TODO remove this check
            print("**************************************** ANGLE DELTA NEGATIVE! *********************")
        
        self.last_angle_rad = current_angle_rad
        self.turn_remaining_rad = self.turn_remaining_rad - turn_progress
        #print("DBG check_turn_complete: turn_progress = ", turn_progress)
        #print("DBG check_turn_complete: turn_remaining_rad = ", self.turn_remaining_rad)

        # rospy.loginfo('%s: Turn Remaining: %02.4f rad, %02.2f degrees' % 
        #    (self.module_name, self.turn_remaining_rad, math.degrees(self.turn_remaining_rad)))
 
        if self.turn_remaining_rad <= TURN_STOP_FUDGE:   
            rospy.loginfo('%s: Rotation Completed.' % self.module_name)
            self.stop()
            return True
            
        if (self.goal_turn_speed != self.slow_turn_speed):
            if (self.turn_remaining_rad <= (math.radians(25.0))): # degrees
                # reduce speed at end of turn
                self.goal_turn_speed = self.slow_turn_speed
                rospy.loginfo('%s: Reducing Turn Speed.' % self.module_name)

        self.publish_turn_command(self.goal_turn_speed)
        
        return False    


    def publish_turn_command(self, speed):
        print("DBG:publish_turn_command: Speed = ", speed)
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = speed
        #print("DBG: sending TWIST command: ", twist)
        self.pub_wheel_motors.publish(twist)
    
    def stop(self):
        # stop any wheel motion
        self.goal_turn_speed = 0.0
        self.publish_turn_command(0.0)
        print("TURN STOP!")
        
    def turn_cleanup(self):
        # un-mute the microphone
        # self.mic_system_enable_pub.publish(True)
        self.stop()
        rospy.loginfo('%s: turn complete' % self.module_name)

        

