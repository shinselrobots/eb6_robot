#!/usr/bin/env python3

import rospy
import logging
import time
import math
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
DEBUG_WRITE_CHAT_LOG_TO_FILE = True
DEBUG_CHAT_LOG_PATH =  '/home/system/Downloads/ai_chat_log.txt'   
MAX_TRACKING_WAIT_TIME = 3.0 # seconds to wait before resetting conversation  
  
# AI Chat State Machine
class State():
    Disabled = "Disabled"
    Ready = "Ready"
    Engage = "Engage"
    Listen = "Listen"
    ListenWait = "ListenWait"
    TalkToAI = "TalkToAI"
    AIResponse = "AIResponse"
    BehaviorWait = "BehaviorWait"
    
class Utilities():
    def __init__(self):
        self.status_pub = rospy.Publisher('/system_status', SystemStatus, queue_size=8)
        
        self.speech_client = None
        if True:  # disable this for testing without speech enabled
            # Start up speech server 
            rospy.loginfo("ai_chat:Utilities: Waiting for speech server...")
            self.speech_client = actionlib.SimpleActionClient("/speech_service", robot_voice.msg.speechAction)
            self.speech_client.wait_for_server()
            rospy.loginfo("ai_chat:Utilities: Speech server ready.")

    def send_status_update(self, item, status):
        # Send status update to system monitor
        status_msg = SystemStatus()
        status_msg.item = item
        status_msg.status = status
        self.status_pub.publish(status_msg)

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

        # Start up on BehaviorWait, until behavior server releases the wait
        self.state = State.BehaviorWait 
        self.last_state = State.BehaviorWait
        self.send_status_update = Utilities().send_status_update
        self.speak = Utilities().speak
        self.ai_interface = AiInterface()
        self.command_intent = CommandIntent()
        
        self.ai_enabled = True        
        self.tracking_person = False 
        self.person = None
        self.known_person_name = ''
        self.conversations = []
        self.listen_timed_out = False
        self.person_said = ""
        self.ai_response = ""
        ##self.last_ai_response = ""
        self.initial_engagement = True
        self.conversation_dump_file = None
        self.tracking_timeout_start = None
        self.system_status_message_to_speak = ''
        self.talk_phrase_number = 0
        self.sleep_phrase_number = 0
        
        # PUBLISHERS
        self.pub_ai_status = rospy.Publisher('/ai_status', UInt16, queue_size=10)
        self.pub_listen = rospy.Publisher('/speech_recognition/listen', Bool, queue_size=5)

        # Command to run a behavior. AI will pause until the behavior completes
        self.behavior_cmd_pub = rospy.Publisher('behavior/cmd', CommandState, queue_size=10)


        # SUBSCRIBERS

        # True = wait while behavior is running, False = continue chat
        rospy.Subscriber("/behavior/wait", Bool, self.behavior_wait_callback)

        # Gets results from streaming speech recognition
        rospy.Subscriber("/speech_recognition/results", String, self.listen_callback)

        # Enable system to report status, without interrupting the conversation too much
        system_status_message_sub = rospy.Subscriber('/status/speak', String, self.system_status_message_callback)

        # Allow Ai to be enabled/disabled by user or sleep/wake behaviors
        rospy.Subscriber("/ai_enable", Bool, self.ai_enable_callback)

        rospy.Subscriber("/person/tracking", Bool, self.person_tracking_state_callback)


        rospy.Subscriber("/person/name", String, self.person_name_callback)
 

        # Final Initialization


        rospy.loginfo("%s: Initialization complete." % (self.logname))     

        # End of Init
        

    # ==========================================================================
    # Callback Functions

    def ai_enable_callback(self, data):
        self.ai_enabled = data.data

        if self.ai_enabled: # state machine will pick this up and run
            rospy.loginfo(self.logname + "========================================")
            rospy.loginfo(self.logname + "        AI Enabled")
            rospy.loginfo(self.logname + "========================================")

        else:
            self.state = State.Disabled  # Force state machine to Disabled state
            self.reset_conversation(True) # Force conversation reset
            rospy.loginfo(self.logname + "========================================")
            rospy.loginfo(self.logname + "        AI Disabled")
            rospy.loginfo(self.logname + "========================================")


    def person_tracking_state_callback(self, data):
        self.tracking_person = data.data
        if self.tracking_person:
            rospy.loginfo(self.logname + "        Tracking Person")
        else:
            rospy.loginfo(self.logname + "        Not Tracking Person")

    def person_name_callback(self, data):
        self.known_person_name = data.data
        print()
        rospy.loginfo(self.logname + "========================================")
        rospy.loginfo(self.logname + "        Got Person Name: " + self.known_person_name)
        rospy.loginfo(self.logname + "========================================")
        print()
        self.send_status_update('AI_NAME', self.known_person_name)

    def listen_callback(self, data):
        # handle response from speech recognition
        self.person_said = data.data
        
        if self.state == State.BehaviorWait:
            return # ignore any pending response from user if already in behavior wait state
        
        if self.person_said == '':
            # listen timed out
            print()
            rospy.loginfo(self.logname + "========================================")
            rospy.loginfo("%s: LISTEN TIMED OUT!" % (self.logname))
            rospy.loginfo(self.logname + "========================================")
            self.listen_timed_out = True
            self.set_state(State.Ready)
        else:
            print()
            rospy.loginfo(self.logname + "========================================")
            rospy.loginfo("%s: Person Said: [%s]" % (self.logname, self.person_said))
            rospy.loginfo(self.logname + "========================================")
            
            # Spot Firm Commands from human
            # Eg, Do you want to talk with me?"  "No." Or "stop talking"
            person_said_lower = self.person_said.lower()
            if ((self.initial_engagement and person_said_lower.find('no') != -1) or 
                (person_said_lower.find('stop talking') != -1)):
                print("DBG:AI:Intent ******************* GOT NEGATIVE RESPONSE FROM HUMAN ********************")
                # person does not want to talk. Ask behavior server to put us in 
                # "pause".  It will switch speech reco to keyword mode and send us a 
                # behavior_wait completion when person says the robot's name
                if self.talk_phrase_number == 0:
                    self.speak("Ok, just say my name if you want to talk.")
                    self.talk_phrase_number = self.talk_phrase_number + 1
                elif self.talk_phrase_number == 1:
                    self.speak("Ok, my lips are sealed, Well, they would be if I had lips.")
                    self.talk_phrase_number = self.talk_phrase_number + 1
                else:
                    self.speak("Ok. I will just hang out.")
                    self.talk_phrase_number = 0
                self.person_said = ''
                self.send_behavior_command("AI_MODE", param1='PAUSE_TALKING', param2='')
                self.set_state(State.BehaviorWait)                         

            elif (person_said_lower.find('go to sleep') != -1):
                print("DBG:AI:Intent ******************* GOT SLEEP COMMAND FROM HUMAN ********************")
                if self.sleep_phrase_number == 0:
                    self.speak("Ok, this will save my batteries")
                    self.sleep_phrase_number = self.sleep_phrase_number + 1
                elif self.sleep_phrase_number == 1:
                    self.speak("My servos are a bit tired. It will be nice to rest up.")
                    self.sleep_phrase_number = self.sleep_phrase_number + 1
                else:
                    self.speak("Good idea. Time for a nap.")
                    self.sleep_phrase_number = 0
                self.send_behavior_command("SLEEP", param1='', param2='')
                self.set_state(State.BehaviorWait)                         
                
            else:   
                self.initial_engagement = False # got initial response from person, so clear this flag
                self.send_status_update('AI_STATE', 'ENGAGED')
                self.set_state(State.TalkToAI)


 
    def keyword_callback(self, data):
        # handle response from keyword recognition
        self.person_said = data.data
        
        if self.person_said == '':
            # keyword only
            rospy.loginfo(self.logname + "========================================")
            rospy.loginfo("%s: KEYWORD ONLY received" % (self.logname))
            rospy.loginfo(self.logname + "========================================")
        else:
            rospy.loginfo(self.logname + "========================================")
            rospy.loginfo("%s: KEYWORD Heard: [%s]" % (self.logname, self.person_said))
            rospy.loginfo(self.logname + "========================================")

            # Now, send the command to the intent handler
            # if valid command, the parser will send it to the behavior server and return True
            rospy.loginfo("%s: Got a Keyword Command... Sending to Intent Parser..." % (self.logname))
            found_command = self.command_intent.handle_command(cmd_string)
            # found command flag is ignored in this case

    def system_status_message_callback(self, data):
        # Speak system status (such as "battery low") at convenient times to minimize interruption
        self.system_status_message_to_speak = data.data

    
    def behavior_wait_callback(self, data):
        wait = data.data
        if wait:
            # Request to wait until a behavior completes
            # Force to wait state immediately. (Example: sent by Sleep behavior)
            self.set_state(State.BehaviorWait)
        else:
            # Behavior complete, so AI Chat can continue             
            self.set_state(State.Ready)


    # ==========================================================================
    # Other Functions


    def append_conversation(self, new_entry):

        self.conversations.append(new_entry)
    
        if DEBUG_WRITE_CHAT_LOG_TO_FILE and self.conversation_dump_file != None:
           print(new_entry, file = self.conversation_dump_file)

    
    def send_behavior_command(self, command, param1='', param2=''):
        print()
        rospy.loginfo("%s: ================================================" % (self.logname))
        rospy.loginfo("%s: Sending behavior command: %s parm1: %s parm2: %s" % 
            (self.logname, command, param1, param2))
        rospy.loginfo("%s: ================================================" % (self.logname))
        msg = CommandState()
        msg.commandState = command
        msg.param1 = param1
        msg.param2 = param2
        self.behavior_cmd_pub.publish(msg)


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
            self.send_status_update('AI_STATE', 'READY') # not ENGAGED
            self.initial_engagement = True
            self.listen_timed_out = False

            # Reset the conversation history
            self.conversations = []

            if DEBUG_WRITE_CHAT_LOG_TO_FILE and self.conversation_dump_file != None:
               print("", file = self.conversation_dump_file)
               print("******* CLEARED CONVERSATION ********", file = self.conversation_dump_file)
               print("", file = self.conversation_dump_file)

    
    def speak_engagement(self):
        # TODO Add random phrases in this function!
        
        if self.initial_engagement:
            if self.listen_timed_out:
                self.speak("Sorry " + self.known_person_name + ", I didnt hear you. Would you like to talk with me?")
            else:        
                self.speak("Hello " + self.known_person_name + ",  would you like to talk with me?")
            
        else:
            # not initial engagement
            if self.listen_timed_out:
                # timed out during conversation
                self.speak("Sorry, " + self.known_person_name + ", I didnt hear you.")
                # Repeat last thing robot said - DISABLED, too annoying!
                ##if self.last_ai_response != "":
                ##    self.speak(self.last_ai_response)
                
            else:
                # happens when returning from a behavior
                self.speak(self.known_person_name + " Whats next?")
                    
        self.listen_timed_out = False


    def set_state(self, new_state):
        # make sure disable did not come in while we were processing
        if self.state != State.Disabled:
            self.state = new_state
            rospy.loginfo("%s: ================================================" % (self.logname))
            rospy.loginfo("%s: New State: %s" % (self.logname, self.state))
            rospy.loginfo("%s: ================================================" % (self.logname))

    def speak_any_system_status(self):
        # Speak system status (such as "battery low") at convenient times to minimize interruption
        if self.system_status_message_to_speak != '':
            self.speak(self.system_status_message_to_speak)  
            self.system_status_message_to_speak = ''  

    def get_ai_response(self):
        # First, check if there are any status messages to speak (this is a good time to do so)
        self.speak_any_system_status()
            
        # Now, add what the person said to the conversation
        self.append_conversation({"role": "user", "content": self.person_said})
        
        # Then send it to the AI and get a response
        self.ai_response = self.ai_interface.conversation(
            self.known_person_name, self.conversations)
        

    def handle_ai_response(self):
        found_command = False
        
        if self.ai_response == None or self.ai_response == "":
            rospy.loginfo("%s: Bad response from AI, Ignored." % (self.logname))
        else:
            rospy.loginfo("%s: ================================================" % (self.logname))
            rospy.loginfo("%s: AI RESPONSE: %s" % (self.logname, self.ai_response))
            rospy.loginfo("%s: ================================================" % (self.logname))

            self.append_conversation({"role": "assistant", "content": self.ai_response})


            # See if the AI found any commands. (AI embeds them between square brackets)
            cmd_string = ''
            speak_string = ''
            (cmd_string, speak_string) = self.command_intent.detect_command(self.ai_response)

            if cmd_string == '':            
                self.speak(self.ai_response)
                ##self.last_ai_response = self.ai_response # in case we want to repeat ourself

                
            else:
                # AI found a command!
                # Speak the non-command part from the AI immediately
                self.speak(speak_string)
                ## self.last_ai_response = '' # responses with commands are not suitable to repeat
                # Now, send the command to the intent handler
                # if valid command, the parser will send it to the behavior server and return True
                rospy.loginfo("%s: AI thinks it found a Command... Sending to Intent Parser..." % (self.logname))

                found_command = self.command_intent.handle_command(cmd_string)
                
        self.ai_response = ''
        return(found_command)

        

#===============================================================================
#                                        LOOP
#===============================================================================

    def run(self):

        if DEBUG_WRITE_CHAT_LOG_TO_FILE:
            self.conversation_dump_file = open(DEBUG_CHAT_LOG_PATH, 'w')
            print("AI Chat Log", file = self.conversation_dump_file)
    
        looprate = rospy.Rate(4) # Loops per second
        while not rospy.is_shutdown():

            if self.state != self.last_state:
                rospy.loginfo("%s: Current State: %s" % (self.logname, self.state))
                self.last_state = self.state

            if self.state == State.Disabled:
                self.speak_any_system_status()

                if self.ai_enabled:
                    self.state = State.Ready # put back into ready state
                else:
                    print("DBG ai_chat: Waiting with ai_enabled = False")
                    self.send_status_update('AI_STATE', 'DISABLED')
                
            elif self.state == State.Ready:
                self.speak_any_system_status()

                # Wait for face tracking to track a person
                if not self.tracking_person:
                    print("DBG ai_chat: Ready, Not Tracking.")
                    self.send_status_update('AI_STATE', 'READY')
                    # start a timer to allow robot to start tracking person
                    # if no person before timeout, reset conversation
                    if self.tracking_timeout_start == None:
                        self.tracking_timeout_start = time.time()
                    else:    
                        tracking_wait_time = time.time() - self.tracking_timeout_start 
                        if tracking_wait_time > MAX_TRACKING_WAIT_TIME:
                            self.tracking_timeout_start == None
                            self.reset_conversation()
                else:
                    self.tracking_timeout_start = None
                    self.set_state(State.Engage)
                
            elif self.state == State.Engage:
                # Person detected, start engaging
                self.speak_engagement()
                self.set_state(State.Listen)
                
            elif self.state == State.Listen:
                self.send_status_update('AI_STATE', 'LISTEN')
                # Send listen command and wait for a response
                self.listen_timed_out = False
                self.pub_listen.publish(True) # one-shot request to listen
                self.set_state(State.ListenWait)
                
            elif self.state == State.ListenWait:
                self.send_status_update('AI_STATE', 'LISTEN_WAIT')
                # we just stay in this state until response from speech recogntion
                # processing happens in listen_callback()
                pass
                
            elif self.state == State.TalkToAI:
                self.send_status_update('AI_STATE', 'TALK_TO_AI')
                self.get_ai_response()
                self.set_state(State.AIResponse)                 
                pass
               
            elif self.state == State.AIResponse:
                self.send_status_update('AI_STATE', 'HANDLE_AI_RESPONSE')
                # process and speak the response
                if self.handle_ai_response():
                    # Found a command. Wait until it completes
                    self.set_state(State.BehaviorWait) 
                    pass
                else:
                    # Keep the conversation going
                    self.set_state(State.Listen) 
                    pass
                
            elif self.state == State.BehaviorWait:
                self.send_status_update('AI_STATE', 'BEHAVIOR_WAIT')
                # we just stay in this state until response from behavior server
                # processing happens in behavior_wait_callback()
                pass
                
            else: 
                rospy.logwarn("%s: ERROR! UNKNOWN STATE! %s" % (self.logname, self.state))
                self.send_status_update('AI_STATE', 'ERROR_UNKNOWN')



            looprate.sleep()  

        if DEBUG_WRITE_CHAT_LOG_TO_FILE:
            self.conversation_dump_file.close()
                
        
        
if __name__ == '__main__':

    node = AiChat()
    node.run()

        
        
        
        
        
        
        

