#! /usr/bin/env python

import rospy
import time
import random
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Empty

# for servos
#from eb_servos.head_servo_publishers import *
#from eb_servos.right_arm_servo_publishers import *
#from eb_servos.left_arm_servo_publishers import *

from eb_servos.set_pose import *
#from eb_servos.standard_servo_positions import *
#from eb_servos.set_servo_speed import *
#from eb_servos.set_servo_torque import *

class HappyBehavior():
    # Make happy movements, such as leaning back and forth

    def __init__(self, behavior_utils, interrupt_check):
        self.module_name = 'happy_behavior'
        rospy.loginfo('%s: Initializing...' % (self.module_name))

        self.interrupt_check = interrupt_check
        self.send_status_update = behavior_utils.send_status_update
        self.speak = behavior_utils.speak
        self.robotpose = RobotPose(self.module_name)

        self.start_thigh_lift = 0.0
        self.start_knee_bend = 0.0
        self.start_ankle_rotate = 0.0
 
 
        self.slider01 = 0.0
        self.slider10 = 0.0

        # Subscribe to debug_sliders for tuning parameters
        slider01_sub  = rospy.Subscriber('/slider/value1',  Float32, self.slider01_cb)
        slider10_sub  = rospy.Subscriber('/slider/value10',  Float32, self.slider10_cb)

        self.pub_light_bar_mode =  rospy.Publisher('/body/strip_mode',   UInt16, queue_size=2) 
        
        rospy.loginfo("%s: init complete." % (self.module_name))

      
    def cleanup(self):
        rospy.loginfo('%s: Cleanup. resetting pose' % self.module_name)
        
        self.pub_light_bar_mode.publish(1) # Return lights to status mode 
               
        # Set Pose back to initial pose
        lowest_servo_speed = 0.3
        goal_pose = self.starting_pose
        self.robotpose.move(goal_pose, lowest_servo_speed)

        rospy.loginfo('%s: Behavior complete' % self.module_name)

    def slider01_cb(self, value):
        self.slider01 = float(value.data)
    def slider10_cb(self, value):
        self.slider10 = float(value.data)



    # ----------------------------------------------------------------
    def execute_behavior(self, param1, param2):
        rospy.loginfo('%s: Executing behavior' % (self.module_name))
        rospy.loginfo( "Param1: '%s'", param1) # Pose
        rospy.loginfo( "Param2: '%s'", param2) # Servo Speed

        sleep_time = 0.25  # seconds
        lean_lowest_servo_speed = 0.3
        lean_amt = 0.12
        #self.speak("Ok")

        self.pub_light_bar_mode.publish(3) # Rainbow
        
        self.starting_pose = self.robotpose.get_current_pose() # save pose to restore it later
        rospy.loginfo('%s: Saving Current pose: [%d]' % (self.module_name, self.starting_pose))
        #(self.start_thigh_lift, self.start_knee_bend, self.start_ankle_rotate) = self.get_leg_servo_positions()
        if self.starting_pose < 2:
            # Happy movements requre a standing pose
            pose_lowest_servo_speed = 0.3 
            goal_pose = 3
            self.starting_pose = goal_pose
            self.robotpose.move(goal_pose, pose_lowest_servo_speed)


        # Possible Optimization: Set servo speeds here instead of in the loop
        # (for example: self.robotpose.set_lean_speeds(lowest_servo_speed))


        for i in range(0, 3):
            if self.interrupt_check():
                rospy.loginfo('%s: Interrupt Detected. Exiting Behavior.' % self.module_name)
                break

            # For tuning: sleep_time = 1.0 + (self.slider01 / 100.0)  # sleep range from 0.0 to 2.0 seconds 
            print("DBG: sleep_time = %0.2f", sleep_time)

            lean_lowest_servo_speed = 0.3 + (self.slider10 / 100.0)  # speed range from 0.3 to 2.3 
            print("DBG: lean servo_speed = %0.2f", lean_lowest_servo_speed)

            direction = 'RIGHT'
            start_time = time.time()
            success = self.robotpose.lean(direction, lean_lowest_servo_speed)
            end_time = time.time()
            elapsed_time = end_time - start_time
            print("Time: {0: 8.4f} ".format(elapsed_time))
            print("********************************************************************************************************")
            
            
            if not success:
                rospy.logwarn('%s: Robotpose.lean error. Exiting Behavior.' % self.module_name)
                break

            rospy.sleep(sleep_time)

            direction = 'LEFT'
            start_time = time.time()
            success = self.robotpose.lean(direction, lean_lowest_servo_speed)
            end_time = time.time()
            elapsed_time = end_time - start_time
            print("Time: {0: 8.4f} ".format(elapsed_time))
            print("********************************************************************************************************")
            if not success:
                rospy.logwarn('%s: Robotpose.lean error. Exiting Behavior.' % self.module_name)
                break
         
            rospy.sleep(sleep_time)

        direction = 'CENTER'

        start_time = time.time()
        success = self.robotpose.lean(direction, lean_lowest_servo_speed)
        end_time = time.time()
        elapsed_time = end_time - start_time
        print("Time: {0: 8.4f} ".format(elapsed_time))
        print("********************************************************************************************************")

        if not success:
            rospy.logwarn('%s: Robotpose.lean error.' % self.module_name)
 
        self.cleanup()         

        
if __name__ == '__main__':
    rospy.init_node('happy_behavior')
    server = HappyBehavior()
    rospy.spin()
    
