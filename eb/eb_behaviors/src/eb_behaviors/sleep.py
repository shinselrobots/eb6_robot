#! /usr/bin/env python

import rospy
import time
#import random
from random import randint

from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import String


# for servos
from eb_servos.head_servo_publishers import *
#from eb_servos.right_arm_servo_publishers import *
#from eb_servos.left_arm_servo_publishers import *

from eb_servos.set_pose import *
#from eb_servos.standard_servo_positions import *
#from eb_servos.set_servo_speed import *
#from eb_servos.set_servo_torque import *

class SleepBehavior():

    def __init__(self, behavior_utils, interrupt_check):
        self.module_name = 'sleep_behavior'
        rospy.loginfo('%s: Initializing...' % (self.module_name))

        self.interrupt_check = interrupt_check
        self.send_status_update = behavior_utils.send_status_update
        self.speak = behavior_utils.speak

        self.robotpose = RobotPose(self.module_name)
        # Get this behavior's *GLOBAL* parameters. These can be set in the master launch file
        #self.sleep_param = rospy.get_param('/sleep', True)
        #rospy.loginfo('%s: GLOBAL PARAM: sleep = %s', self.module_name, self.sleep_param)

        self.peek_mode = False
        self.peek_servo_torque_enabled = False # when doing peek, turn on servos to allow wheel movement too
        rospy.loginfo("%s: init complete." % (self.module_name))


        # PUBLISHERS
        self.ai_enable_pub = rospy.Publisher('/ai_enable', Bool, queue_size=10)
        self.speech_reco_mode_pub = rospy.Publisher('/speech_recognition/mode', String, queue_size=10)
        self.pub_face_tracking_enabled = rospy.Publisher('/head/face_tracking_enabled', Bool, queue_size=4) 
        self.pub_random_move_enabled =  rospy.Publisher('/head/random_move_enabled', Bool, queue_size=4) 

        self.pub_ear_cmd = rospy.Publisher('/head/ear_cmd', UInt16, queue_size=10)        
        self.pub_eye_cmd = rospy.Publisher('/head/eye_cmd', UInt16, queue_size=10)        


      
    def peek_cb(self, data):
        rospy.loginfo('%s: Got PEEK command' % self.module_name)
        print(data)
        if data.data:
            rospy.loginfo('%s: Got PEEK ON command' % self.module_name)
            self.peek_mode = True
        else:    
            rospy.loginfo('%s: Got PEEK OFF command' % self.module_name)
            self.peek_mode = False


    def peek_pose(self, head_up):

        if False:        
            # High Peek
            pub_head_tilt.publish(-0.6)   
            pub_neck_raise.publish(-0.7)    

            # Mid Peek
            pub_head_tilt.publish(-0.9)   
            pub_neck_raise.publish(-1.0)    

            # Mid-low Peek
            pub_head_tilt.publish(-1.2)   
            pub_neck_raise.publish(-1.3)    

            # Low Peek
            pub_head_tilt.publish(-1.2)   
            pub_neck_raise.publish(-1.1)    
        
        if head_up:
            rospy.loginfo('%s: Peek Head UP' % self.module_name)
            if False: # not self.peek_servo_torque_enabled:
                rospy.loginfo("%s:  PEEK TURNING ON SERVOS!" % (self.module_name))
                SetServoTorque(1.0, all_servo_joints)
                self.peek_servo_torque_enabled = True # they stay on (for demos) until new Sleep command resets this
           
            self.pub_eye_cmd.publish(2)     # 2 = Turn eyes on, normal blink mode
            SetServoSpeed(0.5, head_joints)
            pub_head_tilt.publish(-1.0)     # just tilt the head up (-1.3 is higher tilt)
            
        else:
            rospy.loginfo('%s: Peek Head DOWN' % self.module_name)
            self.pub_eye_cmd.publish(0)     # 0 = Turn eyes off
            SetServoSpeed(0.5, head_joints)
            pub_head_tilt.publish(-1.49)    # just tilt the head down 

        pub_neck_raise.publish(-1.48)  # Sleep Position
        pub_head_pan.publish(0.0)      # Sleep Position
        pub_head_sidetilt.publish(0.0) # Sleep Position
        pub_head_pan.publish(0.0)      # Sleep Position 
        

    def cleanup(self):
        rospy.loginfo('%s: Behavior complete' % self.module_name)


    def execute_behavior(self, param1, param2):
        rospy.loginfo('%s: Executing behavior' % (self.module_name))
        rospy.loginfo( "Param1: '%s'", param1)
        rospy.loginfo( "Param2: '%s'", param2)
        
        # peek_mode means robot will occasionally raise head and turn on eyes
        self.peek_mode = False
        peek_sub = rospy.Subscriber("/head_peek", Bool, self.peek_cb)
        pub_head_pan.publish(0.0)      # Pan head to Sleep Position early
        

        # Force AI to off. Behavior server tracks desired AI state, 
        # and Wake only reenables if user had not turned it off
        print()
        rospy.loginfo("%s: *****************************************************" % (self.module_name))
        rospy.loginfo("%s: Disabling AI and putting Speech Reco in Keyword mode." % (self.module_name))
        rospy.loginfo("%s: *****************************************************" % (self.module_name))
        self.ai_enable_pub.publish(False) # turn off AI

        # Set Speech recognition mode to one of: 'disable':'keyword':'stream':
        self.speech_reco_mode_pub.publish('keyword') # Listen for keywords only

        # turn off face tracker movements
        self.pub_face_tracking_enabled.publish(False)
        self.pub_random_move_enabled.publish(False)
        rospy.sleep(0.200) # (seconds) make sure face tracker gets the message


        # EB: Move head and legs to sleep position
        rospy.loginfo("%s:  moving head and legs into sleep position..." % (self.module_name))
        goal_pose = 0 # sleep
        lowest_servo_speed = 0.4
        self.robotpose.move(goal_pose, lowest_servo_speed)

        # allow time for the motion to complete??
        rospy.sleep(2.0) # seconds

        # Turn off lights and servo torque
        rospy.loginfo("%s:Turning off eyes and servo torque" % (self.module_name))
        self.pub_ear_cmd.publish(0) # 0 = Turn ear lights off
        self.pub_eye_cmd.publish(0) # 0 = Turn eyes off

        # Send status update to keep body LEDs off
        # NOTE: Does not effect Head LEDs
        self.send_status_update('BODY_LIGHT_MODE', 'OFF')


        rospy.sleep(1.0) # seconds

        rospy.loginfo("%s:  TURNING OFF SERVOS!" % (self.module_name))
        SetServoTorque(0.0, all_servo_joints)
        
        rospy.loginfo('%s: Sleep Mode Active. Spinning to prevent Idle from running...' % (self.module_name))
        rospy.loginfo("%s: *****************************************************" % (self.module_name))
        print()

        # Run forever to keep Idle behavior from running.
        # may be prempted by any other behavior (such as wake)
        
        SLEEP_TIME = 0.25 #seconds
        peek_counter = 0
        head_is_up = False
        while not rospy.is_shutdown():
            if self.interrupt_check():
                rospy.loginfo('%s: Interrupt Detected. Exiting Behavior.' % self.module_name)
                break
                
            if self.peek_mode:
                self.peek_pose(True) # Move head up
                # Set counter for how long to wait before moving head down
                peek_counter =  3.0 / SLEEP_TIME # seconds
                head_is_up = True
                self.peek_mode = False
            
            if head_is_up:
                # head currently up. When timer expires, bring it down
                peek_counter = peek_counter - 1
                if peek_counter <= 0:
                    self.peek_pose(False) # Move head down       
                    peek_mode = False
                    head_is_up = False
                
            rospy.sleep(SLEEP_TIME) # seconds

        self.cleanup()         

        
if __name__ == '__main__':
    rospy.init_node('sleep_behavior')
    server = SleepBehavior(None)
    rospy.spin()
