#!/usr/bin/env python3
# This test is a hacked up version of the AI state machine code
# Used for testing prompts to the AI

import rospy
import logging
import time
import math
import sys

#import random
#from random import randint

# for talking
import actionlib
import actionlib.action_client
import robot_voice.msg

from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import String
#from geometry_msgs.msg import Point32
from std_msgs.msg import Float32

from system_status_msgs.msg import SystemStatus
from behavior_msgs.msg import CommandState 
from gpt_interface import AiInterface
from intent import CommandIntent


# Global Constants

 
    
class Utilities():

    def speak(self, text_to_speak, wait_for_result = True):
        if self.speech_client:
            speech_goal = robot_voice.msg.speechGoal(text_to_speak)
            rospy.loginfo("Speaking...[%s]" % (text_to_speak))
            self.speech_client.send_goal(speech_goal)
            if wait_for_result:
                result = self.speech_client.wait_for_result() # wait for speech to complete
                rospy.loginfo("Speech goal returned result: %d" % (result))
            else:
                rospy.loginfo("Speech not waiting for result." )


            
# ============================================================================================
class AiChat():
    def __init__(self):
        rospy.init_node('ai_chat')
        #rospy.on_shutdown(self.cleanup)
        self.logname = "ai_chat"
        print("*************** Starting AI Chat **************")
        rospy.loginfo('%s: Starting Node...' % (self.logname))

        self.speak = Utilities().speak
        self.ai_interface = AiInterface()
        # For this test, enable Local YAML in the Intent detector:
        self.command_intent = CommandIntent(True)
        
        self.person = None
        self.known_person_name = ''
        self.conversations = []
        self.person_said = ""
        self.ai_response = ""
        self.initial_engagement = True
        self.conversation_dump_file = None
        self.talk_phrase_number = 0
        self.sleep_phrase_number = 0
        

        rospy.loginfo("%s: Initialization complete." % (self.logname))     

        # End of Init
        

    def append_conversation(self, new_entry):

        self.conversations.append(new_entry)
    

    

    def reset_conversation(self, force_reset=False):
        print("DBG ai_chat: reset_conversation called")
        if force_reset or not self.initial_engagement :
            # We were talking, but lost tracking, so reset
            rospy.loginfo("%s: ================================================" % (self.logname))
            rospy.loginfo("%s: RESETTING CONVERSATION" % (self.logname))
            rospy.loginfo("%s: ================================================" % (self.logname))

            rospy.loginfo("%s: Resetting Conversation" % (self.logname))
            # Forget the focus person we were engaged with
            self.person = None
            self.known_person_name = ''

            # Reset the conversation history
            self.conversations = []

 
    def get_ai_response(self, text_to_send_to_ai):

        # this would come from speech recogniton in the real robot    
        self.person_said = text_to_send_to_ai 
        chat_dump = False
        if self.person_said.find('dump') != -1:
            # remove 'dump' from the input string
            self.person_said.replace('dump', '')
            chat_dump = True
            
            
        # Now, add what the person said to the conversation
        self.append_conversation({"role": "user", "content": self.person_said})

        
        # Then send it to the AI and get a response
        self.ai_response = self.ai_interface.conversation(
            self.known_person_name, self.conversations, chat_dump)
        
        if self.ai_response == None or self.ai_response == "":
            rospy.loginfo("%s: Bad response from AI." % (self.logname))
        else:
            rospy.loginfo("%s: ================================================" % (self.logname))
            rospy.loginfo("%s: AI RESPONSE: %s" % (self.logname, self.ai_response))
            rospy.loginfo("%s: ================================================" % (self.logname))

            # See if the AI found any commands. (AI embeds them between square brackets)
            cmd_string = ''
            speak_string = ''
            (cmd_string, speak_string) = self.command_intent.detect_command(self.ai_response)

            if cmd_string == '':            
                ## DISABLED self.speak(self.ai_response)
                self.append_conversation({"role": "assistant", "content": self.ai_response})
                ##self.last_ai_response = self.ai_response # in case we want to repeat ourself

                
            else:
                # AI found a command!
                # Speak the non-command part from the AI immediately
                rospy.loginfo("%s: AI Parser returned a Speak String: (%s)" % (self.logname, speak_string))
                ## DISABLED self.speak(speak_string)
                ## self.last_ai_response = '' # responses with commands are not suitable to repeat
                self.append_conversation({"role": "assistant", "content": speak_string})

                # Now, send the command to the intent handler
                # if valid command, the parser will send it to the behavior server and return True
                rospy.loginfo("%s: AI Parser returned a Command String: (%s)" % (self.logname, cmd_string))

                found_command = self.command_intent.handle_command(cmd_string)
                

        
    def run(self, text_to_send):


        if text_to_send != "":

            # just send one query and exit
            print()    
            self.get_ai_response(text_to_send)
            return

        # otherwise, enter a question answer loop
        while not rospy.is_shutdown():

            print()    
            question = input("Question: ")
            if question == '':
                print("Exiting...")
                return
                
            self.get_ai_response(question)
               
        
        
if __name__ == '__main__':


        
    total_args = len(sys.argv)
    cmdargs = str(sys.argv)
    text_to_send = ""
    if total_args > 1:
        text_to_send = sys.argv[1]
        print("Text to send = ", text_to_send)
    

    node = AiChat()
    node.run(text_to_send)
        
        
        
        
        

