#! /usr/bin/env python

# Programmed Moves for set distance and/or turn amount
# SimpleMove Command Parmeters:
#   move_amount    # Distance to move in meters. Negative = move backward
#   move_speed     # percent of max robot move speed. Always positive
#   turn_amount    # Amount of turn in radians. Negative = turn left?
#   turn_speed     # percent of max robot turn speed. Always positive 


import rospy
import logging
import time
import math
import sys

# ROS Angles package
#includes: fabs, shortest_angular_distance, normalize_angle_positive, etc.
from angles import * # Python dir() to see list



#from sensor_msgs.msg import JointState
from simple_move_msgs.msg import SimpleMove
from std_msgs.msg import Float32
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from system_status_msgs.msg import SystemStatus

# Odometry
#import tf
#from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Point32, Pose, Quaternion, Twist, Vector3

#from dynamixel_controllers.srv import TorqueEnable, SetServoTorqueLimit, SetSpeed
#from eb_servos.set_pose import *    # for RobotPose
#from eb_servos.servo_joint_list import all_servo_joints, head_joints, right_leg_joints, left_leg_joints
#from eb_servos.head_servo_publishers import *
#from eb_servos.leg_servo_publishers import *

#from eb_servos.standard_servo_positions import *
#from eb_servos.set_servo_speed import *
#from eb_servos.set_servo_torque import *
#from eb_servos.usb_latency_test import *
#from eb_servos.srv import ReturnJointStates

OBJECT_COLLISION_DISTANCE = 0.190 # 180mm is min detectable range

#-------------------------------------------------------------------------------    
class ProgrammedMove():
    # Handle requested "simple_move" commands, to move defined distances
    def __init__(self, collision_slow_distance, collision_stop_distance_front, collision_stop_distance_rear):    
        self.module_name = 'programmed_move'
        rospy.loginfo('WheelControl:%s: Initializing...' % (self.module_name))


        # Constants
        self.DEFAULT_MOVE_SPEED = 0.400 
        self.MIN_MOVE_SPEED = 0.200
        self.MOVE_STOP_FUDGE = 0.010 # about 4 inches
        self.MOVE_SLOW_DISTANCE = 0.500  # meters

        self.DEFAULT_TURN_SPEED = 0.50 # half max
        self.MIN_TURN_SPEED = 0.20
        self.TURN_STOP_FUDGE = 0.080 # radians
        self.MIN_MOVE_DISTANCE = 0.09 # Note: moves below 0.25 will be inaccurate
        self.MIN_TURN_AMOUNT = 5.0 # degrees

        # Variables
        self.move_in_progress = False
        self.turn_in_progress = False

        self.goal_move_speed = 0.0
        self.goal_move_distance = 0.0
        self.slow_move_speed = self.MIN_MOVE_SPEED # slow down at the end for precision
        self.current_odom = 0.0
        self.last_odom = 0.0
        self.odom_received = False

        self.goal_turn_speed = 0.0
        self.turn_remaining_rad = 0.0
        self.slow_turn_speed = self.MIN_TURN_SPEED # slow down at the end for precision
        self.imu_yaw = 0.0
        self.last_angle_rad = 0.0
        self.imu_received = False
        self.sensor_summary = None
        self.collision_prevention_enabled = True
        self.collision_slow_distance = collision_slow_distance
        self.collision_stop_distance_front = collision_stop_distance_front
        self.collision_stop_distance_rear = collision_stop_distance_rear

        # Override battery indicators if an object blocks movement        
        self.pub_body_strip_color = rospy.Publisher('/body/strip_status_color', UInt32, queue_size=10)
        self.status_pub = rospy.Publisher('/system_status', SystemStatus, queue_size=6)
        

    def odom_callback(self, new_odom):
        # gets updates from WheelControl on odom updates
        self.current_odom = new_odom
        self.odom_received = True 
        #print("%s: Odom Update: %8.4f" % (self.module_name, self.current_odom))


    def imu_callback(self, data): # Degrees
        #print("got imu: ", data) 
        #self.imu_roll = data.x
        #self.imu_pitch = data.y
        self.imu_yaw = data.z
        self.imu_received = True

    def send_status_update(self, item, status):
        # Send status update system monitor
        status_msg = SystemStatus()
        status_msg.item = item
        status_msg.status = status
        self.status_pub.publish(status_msg)


    def sensor_callback(self, data):
        # print("got sensor data")
        self.sensor_summary = data
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
        
        if not self.collision_prevention_enabled:
            rospy.logwarn("%s:begin:DBG: Collision Prevention DISABLED!" % (self.module_name))
            return

        if not self.move_in_progress:
            # don't do anything if the robot is not doing programmed move (may be turning, that's ok)
            return

        # Check for Front Object - Collision Distance - Stop
        if self.goal_move_speed > 0.0 and self.sensor_summary.nearest_object_front < self.collision_stop_distance_front:
            rospy.logwarn('%s: Object Ahead at %02.4f m! Programmed move cancelled!' % 
                (self.module_name, self.sensor_summary.nearest_object_front))
            self.goal_move_speed = 0.0
            # Led 0 (top LED) to red to show blocked in front (gets cleared at next battery update)
            self.pub_body_strip_color.publish(0x001F0000) 
            
            self.send_status_update('OBJECT_FRONT', 'BLOCKED')

        # Check for Front Object - Further away - Slow down
        elif self.goal_move_speed > 0.0 and self.sensor_summary.nearest_object_front < self.collision_slow_distance:

            if self.goal_move_speed != self.slow_move_speed:
            # Moving forward and object detected ahead
                rospy.logwarn('%s: Object Ahead at %02.4f m! Slowing.' % 
                    (self.module_name, self.sensor_summary.nearest_object_front))
                self.goal_move_speed == self.slow_move_speed


        # Check for Rear Object - Collision Distance - Stop
        elif self.goal_move_speed < 0.0 and self.sensor_summary.nearest_object_rear < self.collision_stop_distance_rear:
            rospy.logwarn('%s: Object Behind at %02.4f m! Programmed move cancelled!' % 
                (self.module_name, self.sensor_summary.nearest_object_rear))
            self.goal_move_speed = 0.0
            # Led 1 to red to show blocked in rear (gets cleared at next battery update)
            self.pub_body_strip_color.publish(0x011F0000) 

            self.send_status_update('OBJECT_REAR', 'BLOCKED')

        # Check for Rear Object - Further away - Slow down
        elif self.goal_move_speed < 0.0 and self.sensor_summary.nearest_object_rear < self.collision_slow_distance:
            if self.goal_move_speed != self.slow_move_speed:
            # Moving forward and object detected ahead
                rospy.logwarn('%s: Object Behind at %02.4f m! Slowing.' % 
                    (self.module_name, self.sensor_summary.nearest_object_rear))
                self.goal_move_speed == self.slow_move_speed

        

    def check_for_objects_before_start(self):
        if self.sensor_summary == None:
            return

        if self.goal_move_speed > 0:
            # Moving forward
            if self.sensor_summary.nearest_object_front < self.collision_stop_distance_front:
                rospy.logwarn("%s: Begin: Front Object detected at %2.2f meters. Linear Move cancelled" % 
                    (self.module_name, self.sensor_summary.nearest_object_front))
                self.goal_move_speed = 0.0 # cancel move
                self.goal_move_distance = 0.0 
                #self.turn_remaining_rad = 0.0 # TODO cancel turn???

                # Led 8 (bottom LED) to red to show blocked in front (gets cleared at next battery update)
                self.pub_body_strip_color.publish(0x801F0000) 

                self.send_status_update('OBJECT_FRONT', 'BLOCKED')

        elif self.goal_move_speed < 0:
            # Moving backward
            if self.sensor_summary.nearest_object_rear < self.collision_stop_distance_rear:    
                rospy.logwarn("%s: Begin: Rear Object detected at %2.2f meters. Move cancelled" % 
                    (self.module_name, self.sensor_summary.nearest_object_rear))
                self.goal_move_speed = 0.0 # cancel move
                self.goal_move_distance = 0.0 
                #self.turn_remaining_rad = 0.0 # TODO cancel turn???

                # Led 7 to red to show blocked in rear (gets cleared at next battery update)
                self.pub_body_strip_color.publish(0x401F0000) 

                self.send_status_update('OBJECT_REAR', 'BLOCKED')

 
    # ==========================================================================================================   
    def begin(self, msg, collision_prevention_enabled = True):

        rospy.loginfo("%s:begin: move_amount: %2.2f, move_speed: %2.2f turn_amount: %2.2f, turn_speed: %2.2f" % 
            (self.module_name, msg.move_amount, msg.move_speed, msg.turn_amount, msg.turn_speed))
        self.collision_prevention_enabled =  collision_prevention_enabled
           
        if self.collision_prevention_enabled:
            rospy.loginfo("%s:begin: Collision Prevention Enabled" % (self.module_name))
        else:
            rospy.logwarn("%s:begin: Collision Prevention DISABLED!" % (self.module_name))
            
        move_amount = msg.move_amount       # in meters
        move_speed =  fabs(msg.move_speed)  # requested speeds should always be positive
        turn_amount_deg = msg.turn_amount   # In degrees
        turn_speed =  fabs(msg.turn_speed)  # requested speeds should always be positive
        
        rospy.loginfo("requested distance:    %2.2f meters" % move_amount) # # distance in meters (positive = forward)
        rospy.loginfo("requested speed:       %2.2f percent" % move_speed) # move speed from MIN_MOVE_SPEED to 1.0 (always positive)
        rospy.loginfo("requested turn amount  %2.2f degrees" % turn_amount_deg) # +/- turn amount in degrees. Positive = left
        rospy.loginfo("requested turn speed:  %2.2f percent" % turn_speed) # Turn speed from MIN_TURN_SPEED to 1.0 (always positive)

        self.reset() # initialize variables

        # Confirm Odom is reporting position
        if not self.odom_received:
            rospy.logwarn('%s: No ODOM data received! Programmed move cancelled!' % (self.module_name))
            return False, 0.0, 0.0

        # Confirm IMU is reporting position
        if self.imu_yaw == 0.0:
            rospy.logwarn('%s: No IMU data received! Programmed move cancelled!' % (self.module_name))
            return False, 0.0, 0.0

        if False: #self.sensor_summary == None:
            rospy.logwarn('%s: No Collision Sensor data received! Programmed move cancelled!' % (self.module_name))
            return False, 0.0, 0.0


        # ================== LINEAR MOVES ================
        if fabs(move_amount) < self.MIN_MOVE_DISTANCE: 
            # Ignore small moves. Just a turn
            self.goal_move_distance = 0.0
            self.goal_move_speed = 0.0
            if move_amount != 0.0:
                rospy.logwarn('%s: Requested move amount [%2.2f] < MIN_MOVE_DISTANCE. Ignored!' % 
                    (self.module_name, fabs(move_amount)))

        else:
            # FORWARD SPEED
            if move_speed <  self.MIN_MOVE_SPEED: 
                rospy.logwarn('%s: Requested move speed [%2.2f] < MIN_MOVE_SPEED! SPEED SET TO MIN [%2.2f]' % 
                    (self.module_name, move_speed, self.MIN_MOVE_SPEED))
                move_speed = self.MIN_MOVE_SPEED

            # apply move direction to speed command
            self.slow_move_speed = self.MIN_MOVE_SPEED
            if move_amount < 0.0:
                move_speed = move_speed * -1.0
                self.slow_move_speed = self.slow_move_speed * -1.0
                rospy.loginfo('%s: Moving backward' % (self.module_name))
            self.goal_move_speed = move_speed

            # FORWARD DISTANCE
            self.goal_move_distance = fabs(move_amount) # move has no sign
            
        # Get initial position odometry
        self.start_odom = self.current_odom
        rospy.loginfo("%s: Start Position: %2.2f" % (self.module_name, self.start_odom))


        # ================== TURNS ================
        if fabs(turn_amount_deg) < self.MIN_TURN_AMOUNT:
            # no turn, just a linear move
            self.turn_remaining_rad = 0.0
            self.goal_turn_speed = 0.0
            if turn_amount_deg != 0.0:
                rospy.logwarn('%s: Requested turn amount [%2.2f] < MIN_TURN_AMOUNT! Ignored.' % 
                    (self.module_name, turn_amount_deg))

        else:

            # TURN SPEED
            if turn_speed <  self.MIN_TURN_SPEED:
                rospy.logwarn('%s: Requested turn speed [%2.2f] < MIN_TURN_SPEED! SPEED SET TO MIN [%2.2f]' 
                    % (self.module_name, turn_speed, self.MIN_TURN_SPEED))
                turn_speed = self.MIN_TURN_SPEED

            # apply turn direction to turn speed sign
            self.slow_turn_speed = self.MIN_TURN_SPEED
            if turn_amount_deg < 0.0:
                turn_speed = turn_speed * -1.0
                self.slow_turn_speed = self.slow_turn_speed * -1.0
            self.goal_turn_speed = turn_speed

            # TURN AMOUNT
            self.turn_remaining_rad = math.radians(fabs(turn_amount_deg)) # turn amount has no sign
            rospy.loginfo("%s: Turn amount requested: %f rad, %f deg" % 
                (self.module_name, self.turn_remaining_rad, turn_amount_deg))
             
        # Get initial rotation position
        self.last_angle_rad = normalize_angle_positive(math.radians(self.imu_yaw))
        rospy.loginfo("%s: Start Angle: %f radians, %f degrees" % 
            (self.module_name, self.last_angle_rad, math.degrees(self.last_angle_rad)))


        # Don't move if objects are too close
        self.check_for_objects_before_start()

        # Begin move
        if self.goal_move_distance != 0.0 and self.goal_move_speed != 0.0:
            self.move_in_progress = True
            rospy.loginfo("%s: Begin: Move speed = %2.2f" % 
                (self.module_name, self.goal_move_speed))

        if self.turn_remaining_rad != 0.0: 
            self.turn_in_progress = True
            rospy.loginfo("%s: Begin: Turn speed = %2.2f" % 
                (self.module_name, self.goal_turn_speed))

        init_success = (self.move_in_progress or self.turn_in_progress)

        
        return init_success, self.goal_move_speed, self.goal_turn_speed       


 
    def update(self):
        # return updates for programmed move, including speed changes needed
        
        if (not self.move_in_progress) and (not self.turn_in_progress):
            # No moves in progress
            return False, 0.0, 0.0


        # Check MOVE progress
        if self.move_in_progress and self.goal_move_speed != 0.0 and self.goal_move_distance != 0.0:
            move_progress = self.current_odom - self.start_odom
            #print("DBG check_move_complete: move_progress = ", move_progress)
            remaining = self.goal_move_distance - fabs(move_progress)
            rospy.loginfo('%s: Move Remaining: %02.4f' % (self.module_name, remaining))
     
            if remaining <= self.MOVE_STOP_FUDGE:   
                rospy.loginfo('%s: Completing Move' % self.module_name)
                # dont do here self.move_in_progress = False
                self.goal_move_speed = 0.0
                
            elif self.goal_move_speed != self.slow_move_speed:
                if remaining <= self.MOVE_SLOW_DISTANCE:
                    # reduce speed at end of move
                    self.goal_move_speed = self.slow_move_speed
                    rospy.loginfo('%s: Reducing Move Speed to %2.2f' % 
                        (self.module_name, self.goal_move_speed))



        # Check TURN progress
        if self.turn_in_progress and self.goal_turn_speed != 0.0 and self.turn_remaining_rad != 0.0:
            current_angle_rad = normalize_angle_positive(math.radians(self.imu_yaw))
            #print("DBG check_turn_complete: current_angle_radians = ", current_angle_rad)

            # handle wrap-around
            turn_progress = fabs(shortest_angular_distance(self.last_angle_rad, current_angle_rad));
            
            self.last_angle_rad = current_angle_rad
            self.turn_remaining_rad = self.turn_remaining_rad - turn_progress
            #print("DBG check_turn_complete: turn_progress      = %02.4f" % turn_progress)
            #print("DBG check_turn_complete: turn_remaining_rad = %02.4f" % self.turn_remaining_rad)

            rospy.loginfo('%s: Turn Remaining: %02.4f rad, %02.2f degrees' % 
                (self.module_name, self.turn_remaining_rad, math.degrees(self.turn_remaining_rad)))
     
            if self.turn_remaining_rad <= self.TURN_STOP_FUDGE:   
                rospy.loginfo('%s: Completing Turn.' % self.module_name)
                # dont do here: self.turn_in_progress = False
                self.goal_turn_speed = 0.0
                
            elif (self.goal_turn_speed != self.slow_turn_speed):
                if (self.turn_remaining_rad <= (math.radians(25.0))): # degrees
                    # reduce speed at end of turn
                    self.goal_turn_speed = self.slow_turn_speed
                    rospy.loginfo('%s: Reducing Turn Speed to %2.2f' % (self.module_name, self.goal_turn_speed))

        # Finalilze
        if (self.move_in_progress or self.turn_in_progress) and (self.goal_move_speed == 0.0 and self.goal_turn_speed == 0.0):
            # Move and Turn completed, but in_progress flag not cleared yet.
            rospy.loginfo('%s: All move and turn Completed.' % self.module_name)            
            #TODO - If needed, publish "all moves completed" message in case someone is wating?
            # Note that return will stop movement, then we're ready for next programmed move.        

        # Handle this here to make sure stop is reported to the calling function
        if self.move_in_progress and self.goal_move_speed == 0.0:
            rospy.loginfo('%s: Move Completed.' % self.module_name)
            self.move_in_progress = False

        if self.turn_in_progress and self.goal_turn_speed == 0.0:
            rospy.loginfo('%s: Turn Completed.' % self.module_name)
            self.turn_in_progress = False
        
        return True, self.goal_move_speed, self.goal_turn_speed 

        
 
    def reset(self):

        self.move_in_progress = False
        self.turn_in_progress = False
        
        self.goal_move_speed = 0.0
        self.slow_move_speed = self.MIN_MOVE_SPEED # slow down at the end for precision
        self.goal_move_distance = 0.0
        
        self.goal_turn_speed = 0.0
        self.slow_turn_speed = self.MIN_TURN_SPEED # slow down at the end for precision
        self.turn_remaining_rad = 0.0
 

 
 
 
 
 
 
 
 
 
    

