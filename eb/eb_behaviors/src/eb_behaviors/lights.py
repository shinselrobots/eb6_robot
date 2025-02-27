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
#from eb_servos.head_servo_publishers import *
#from eb_servos.right_arm_servo_publishers import *
#from eb_servos.left_arm_servo_publishers import *

from eb_servos.set_pose import *
#from eb_servos.standard_servo_positions import *
#from eb_servos.set_servo_speed import *
#from eb_servos.set_servo_torque import *


# Led bar modes
# "STATUS" # System Status (default)
# "MUSIC"  # Displaying Music beats
# "OFF"    # 


class LightsBehavior():

    def __init__(self, behavior_utils, interrupt_check):
        self.module_name = 'lights_behavior'
        rospy.loginfo('%s: Initializing...' % (self.module_name))

        self.interrupt_check = interrupt_check
        self.send_status_update = behavior_utils.send_status_update
        self.speak = behavior_utils.speak
        self.lights_on = False
        # TODO - Get light status from status module?

        # Publishers
        self.pub_ear_cmd =   rospy.Publisher('/head/ear_cmd',   UInt16, queue_size=10)        
        self.pub_eye_cmd =   rospy.Publisher('/head/eye_cmd',   UInt16, queue_size=10)        
        #self.pub_eye_color = rospy.Publisher('/head/eye_color', UInt32, queue_size=10)        
        #self.pub_light_bar = rospy.Publisher('/body/strip_color', UInt32, queue_size=1)
                
        rospy.loginfo("%s: init complete." % (self.module_name))
    

    def cleanup(self):
        rospy.loginfo('%s: Behavior complete' % self.module_name)

    def execute_behavior(self, param1, param2):
        rospy.loginfo('%s: Executing behavior' % (self.module_name))
        rospy.loginfo( "Param1: '%s'", param1)
        rospy.loginfo( "Param2: '%s'", param2)


        if param1 == "TOGGLE":
            if self.lights_on:
                self.lights_on = False
            else:
                self.lights_on = True
        elif param1 == "TRUE" or param1 == "ON":
                self.lights_on = True
        elif param1 == "FALSE" or param1 == "OFF":
            self.lights_on = False
        else:
            rospy.logwarn('%s: Bad Light mode parameter!  Ignored command!' % self.module_name)
            self.cleanup()
            return        


        if self.lights_on:
            rospy.loginfo('%s: Lights On' % self.module_name)
            rand_phrase = randint(1, 4)
            if rand_phrase == 1: 
                self.speak("doesnt this look nice?")
            elif rand_phrase == 2:
                self.speak("this is robot bling")
            else:
                self.speak("the lights are on, but nobodys home")
            self.pub_eye_cmd.publish(2) # 2 = Turn eyes on, normal blink mode
            self.pub_ear_cmd.publish(3) # 3 = Turn ear lights on, rainbow mode
            self.send_status_update('BODY_LIGHT_MODE', 'STATUS')
            self.lights_on = True
                
        else:
            rospy.loginfo('%s: Lights Off.' % self.module_name)
            rand_phrase = randint(1, 4)
            if rand_phrase == 1: 
                self.speak("i am playing it cool")
            elif rand_phrase == 2:
                self.speak("ok, turning my lights off")
            else:
                self.speak("entering stealth mode")
            self.pub_eye_cmd.publish(0) # 0 = Turn eyes off
            self.pub_ear_cmd.publish(0) # 0 = Turn ears off
            self.send_status_update('BODY_LIGHT_MODE', 'OFF')
            self.lights_on = False


        
        self.cleanup()         

        
if __name__ == '__main__':
    rospy.init_node('lights_behavior')
    server = LightsBehavior()
    rospy.spin()
