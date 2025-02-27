#! /usr/bin/env python

import rospy
import time
import random
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import String

from system_status_msgs.msg import SystemStatus

# for servos
#from eb_servos.head_servo_publishers import *
#from eb_servos.right_arm_servo_publishers import *
#from eb_servos.left_arm_servo_publishers import *

from eb_servos.set_pose import *
#from eb_servos.standard_servo_positions import *
#from eb_servos.set_servo_speed import *
#from eb_servos.set_servo_torque import *

DEBUG_DISABLE_POSE = False
DBG_DISABLE_SPEECH = False  # If true, don't wait for speech client when debugging


class WakeBehavior():

    def __init__(self, behavior_utils, interrupt_check, ai_is_enabled):
        self.module_name = 'wake_behavior'
        rospy.loginfo('%s: Initializing...' % (self.module_name))

        self.interrupt_check = interrupt_check
        self.ai_is_enabled = ai_is_enabled
        self.send_status_update = behavior_utils.send_status_update
        self.speak = behavior_utils.speak
        self.robotpose = RobotPose(self.module_name)
        self.first_time_kludge = False
        self.reco_status_good = False 
        self.EAR_CMD_AI_MODE       = 2 # Chasing (thinking)
        self.EAR_CMD_KEYWORD_MODE  = 3 # Rainbow

        # Get this behavior's *GLOBAL* parameters. These can be set in the master launch file
        self.enable_ear_lights = rospy.get_param('/enable_ear_lights', True)
        rospy.loginfo('%s: GLOBAL PARAM: enable_ear_lights = %s', self.module_name, 
            self.enable_ear_lights)

        self.enable_eye_lights = rospy.get_param('/enable_eye_lights', True)
        rospy.loginfo('%s: GLOBAL PARAM: enable_eye_lights = %s', self.module_name, 
            self.enable_eye_lights)

        self.enable_body_lights = rospy.get_param('/enable_body_lights', True)
        rospy.loginfo('%s: GLOBAL PARAM: enable_body_lights = %s', self.module_name, 
            self.enable_body_lights)


        # SUBSCRIBERS
        # check system status when robot is waking up
        # ai_status_sub = rospy.Subscriber('/ai_status', UInt16, self.ai_status_callback)
        self.status_update_sub = rospy.Subscriber("/system_status", SystemStatus, self.status_update_callback)
        
        
        # PUBLISHERS
        self.ai_enable_pub = rospy.Publisher('/ai_enable', Bool, queue_size=10)
        self.speech_reco_mode_pub = rospy.Publisher('/speech_recognition/mode', String, queue_size=10)

        self.pub_ear_mode = rospy.Publisher('/head/ear_cmd', UInt16, queue_size=10)
        self.pub_ear_cmd = rospy.Publisher('/head/ear_cmd', UInt16, queue_size=10)        
        self.pub_eye_cmd = rospy.Publisher('/head/eye_cmd', UInt16, queue_size=10)        
        self.pub_eye_color = rospy.Publisher('/head/eye_color', UInt32, queue_size=10)
        ## TODO REMOVE? self.sleep_mode_pub =   rospy.Publisher('/sleep_mode', Bool, queue_size=10) # for the AI module

        # Send status update to keep all LEDs off
        self.send_status_update('BODY_LIGHT_MODE', 'OFF')


        rospy.loginfo("%s: init complete." % (self.module_name))
 
      
    def status_update_callback(self, status_msg):
        # Gets updates from modules (sent via topic "/system_status")
        item_str = status_msg.item
        status_str = status_msg.status
        #rospy.loginfo("%s: Got Status update [%s] [%s]" % (self.module_name, item_str, status_str))            

        if item_str == 'SPEECH_RECO_STATE': 
            # rospy.loginfo("%s: Got Status update [%s] [%s]" % (self.module_name, item_str, status_str))            
            if status_str == 'MIC_FAIL':
                self.reco_status_good = False
            else:
                self.reco_status_good = True

        # ignore any other messages

    def ai_status_callback(self, data):
        print("****************** Got AI Status update: ", data) 
        self.ai_status = int(data.data)


    def init_leds(self):
        if DEBUG_DISABLE_POSE:
            return # LOCAL OVERRIDE don't turn on LEDs

        if self.enable_eye_lights:
            self.pub_eye_cmd.publish(2) # 2 = Turn eyes on, normal blink mode
        else:
            self.pub_eye_cmd.publish(0) # 0 = Turn eyes off
        
        if self.enable_ear_lights:
            self.pub_ear_cmd.publish(3) # 3 = Turn ear lights on, rainbow mode
        else:
            self.pub_ear_cmd.publish(0) # 0 = Turn ears off
            
        if self.enable_body_lights:
            self.send_status_update('BODY_LIGHT_MODE', 'STATUS')

        else:
            # Send status update to disable status LEDs
            self.send_status_update('BODY_LIGHT_MODE', 'OFF')


    def cleanup(self):
        rospy.loginfo('%s: Behavior complete' % self.module_name)

    def execute_behavior(self, param1, param2):
        rospy.loginfo('%s: Executing behavior' % (self.module_name))
        rospy.loginfo( "Param1: '%s'", param1)
        rospy.loginfo( "Param2: '%s'", param2) # NOT USED


        if not DEBUG_DISABLE_POSE:
                    
            SetServoTorque(1.0, all_servo_joints) # turn on torque to avoid startup jitter
            
            # Turn on body lights right away, so we know the robot heard the command
            # Note that sometimes on power-up, the arduino is not ready yet, so we do it again later
            if self.enable_body_lights:
                # Send status update to turn status LEDs on
                self.send_status_update('BODY_LIGHT_MODE', 'ON')
            else:
                self.send_status_update('BODY_LIGHT_MODE', 'OFF')


        self.speak("initializing system")

        if True: # self.first_time_kludge:
            # tell robot to move a bit before moving to wake position. 
            # Some WEIRD reason, ankles don't work fast enough on the first pose command
            goal_pose = 2 
            lowest_servo_speed = 0.3
            self.robotpose.move(goal_pose, lowest_servo_speed)
            self.first_time_kludge = False


        self.init_leds() # turn on eyes and ears as the head comes up
        rospy.sleep(2.0) # Allow servos to reach first position

        if not DEBUG_DISABLE_POSE:
            # (in debug mode, just leave robot where it was last time)
            
            # Move head and legs to position
            rospy.loginfo("%s:  moving head and legs into position..." % (self.module_name))
            lowest_servo_speed = 0.3
            goal_pose = 3
            self.robotpose.move(goal_pose, lowest_servo_speed)
            time.sleep(3.0) # seconds
            self.send_status_update('FACE_TRACKER', 'WAIT_FIRST_FACE') # initializing, first frame not received


        # say something
        speak_phrase = "all systems ready."
        if not self.reco_status_good:
            speak_phrase = "Error, microphone is offline"
            self.enable_ear_lights = False # Turn off ear lights to indicate that microphone is offline

        self.speak(speak_phrase)

        # Enable speech recognition. Set mode to one of: 'disable':'keyword':'stream':
        print()
        rospy.loginfo("%s: *****************************************************" % (self.module_name))
        rospy.loginfo("%s: Putting Speech Reco in Streaming mode." % (self.module_name))
        self.speech_reco_mode_pub.publish('stream') # normal mode
        

        time.sleep(1.0) # seconds
        # init leds again, because sometimes the arduino is not up for the first call
        # and in case AI is not up
        self.init_leds() 

        if self.ai_is_enabled():       
            # Tell AI that it's OK to start engaging
            rospy.loginfo("%s: Enabling AI." % (self.module_name))
            rospy.loginfo("%s: *****************************************************" % (self.module_name))
            self.ai_enable_pub.publish(True)         
            ##self.pub_ear_mode.publish(self.EAR_CMD_AI_MODE) # Indicate AI mode        
            ## Also enables status light updates
            ## TODO self.sleep_mode_pub.publish(False) 
        else:
            rospy.loginfo("%s: AI Disabled by user. Leaving in disabled state." % (self.module_name))
            rospy.loginfo("%s: *****************************************************" % (self.module_name))


        self.cleanup()          

        
if __name__ == '__main__':
    rospy.init_node('wake_behavior')
    server = WakeBehavior(None)
    rospy.spin()
