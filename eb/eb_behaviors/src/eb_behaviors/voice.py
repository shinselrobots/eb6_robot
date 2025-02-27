#! /usr/bin/env python
# Selects Voice to use

import rospy
import time
import random
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import String
from std_msgs.msg import Empty

# for servos
#from eb_servos.head_servo_publishers import *
#from eb_servos.right_arm_servo_publishers import *
#from eb_servos.left_arm_servo_publishers import *

from eb_servos.set_pose import *
#from eb_servos.standard_servo_positions import *
#from eb_servos.set_servo_speed import *
#from eb_servos.set_servo_torque import *

class SetVoiceBehavior():
    # Simple behavior. Robot just says whatever was requested.
    
    def __init__(self, behavior_utils, interrupt_check):
        self.module_name = 'setvoice_behavior'
        rospy.loginfo('%s: Initializing...' % (self.module_name))
        self.interrupt_check = interrupt_check
        self.send_status_update = behavior_utils.send_status_update
        self.speak = behavior_utils.speak
        
        
        self.voices = ['poppy', 'linda', 'david', 'lessac', 'prudence', 'spike', 'libritts']         
        self.voice_index = 0

        self.set_voice_pub = rospy.Publisher('/voice_name', String, queue_size=2)
        self.set_voice_number_pub = rospy.Publisher('/voice_number', UInt32, queue_size=2)


        
        rospy.loginfo("%s: init complete." % (self.module_name))

    def cleanup(self):
        rospy.loginfo('%s: Behavior complete' % self.module_name)

    def execute_behavior(self, param1, param2):
        rospy.loginfo('%s: Executing behavior' % (self.module_name))
        rospy.loginfo( "Param1: '%s'", param1)
        rospy.loginfo( "Param2: '%s'", param2)

        voice = param1.lower()
        default_voice_announce = ''

        if param1.isdigit():
            # For testing speaker variations in certain voices
            voice_number = 0
            try:
                voice_number = int(param1)
                print("got number", voice_number)
            except:
                rospy.logwarn('%s: Bad value [%s] for voice number. Ignored.' % (self.module_name, param1))
                self.cleanup()
                return
                
            # force to this voice:
            self.set_voice_pub.publish("poppy")
            #self.set_voice_pub.publish("libritts")
            print("set voice")
                
            self.set_voice_number_pub.publish(voice_number)
            print("published number", voice_number)
            
            voice_number_string = "this is speaker " + str(voice_number) + ". "
            self.speak(voice_number_string) 

            optional_text_to_speak = param2 
            if optional_text_to_speak != '' and optional_text_to_speak.lower() != 'x':
                self.speak(optional_text_to_speak) # Say whatever phrase was requested



        elif voice == '' or voice == 'x':
            # just cycle through the voices
            voice_name = self.voices[self.voice_index]
                
            rospy.loginfo('%s: Sending voice selection index %d [%s]' % (self.module_name, self.voice_index, voice_name))    
            self.set_voice_pub.publish(voice_name)    

            if self.voice_index == 0:
                self.speak("default voice") 

            voice_name_string = "this is voice " + voice_name + ". "
            self.speak(voice_name_string) 

            optional_text_to_speak = param2 
            if optional_text_to_speak != '' and optional_text_to_speak.lower() != 'x':
                self.speak(optional_text_to_speak) # Say whatever phrase was requested


            self.voice_index = self.voice_index + 1
            if self.voice_index >= len(self.voices):
                self.voice_index = 0


        self.cleanup()         

        
if __name__ == '__main__':
    rospy.init_node('setvoice_behavior')
    server = SayBehavior(None)
    rospy.spin()
