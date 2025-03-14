#!/usr/bin/env python3
# Behavior system
# Testing example: rostopic pub -1 /behavior/cmd behavior_msgs/CommandState -- "WAKEUP" "x" "x"

import sys
import roslib
import rospy
import signal
import os
import time
#import threading # for music
#from pygame import mixer
import pygame  # pip install pygame


# for talking
import actionlib
import actionlib.action_client
import robot_voice.msg

from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Int16
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import String

from behavior_msgs.msg import CommandState
from system_status_msgs.msg import SystemStatus

from eb_behaviors.example import ExampleBehavior
from eb_behaviors.wake import WakeBehavior
from eb_behaviors.sleep import SleepBehavior
from eb_behaviors.nothing import NothingBehavior
from eb_behaviors.idle import IdleBehavior
from eb_behaviors.turn import TurnBehavior
from eb_behaviors.move import MoveBehavior
from eb_behaviors.pose import PoseBehavior
from eb_behaviors.head_center import HeadCenterBehavior
#from eb_behaviors.ai_mode import AiModeBehavior
from eb_behaviors.happy import HappyBehavior
from eb_behaviors.dance import DanceBehavior
from eb_behaviors.intro import IntroBehavior
from eb_behaviors.joke import JokeBehavior
from eb_behaviors.bad import BadBehavior
from eb_behaviors.voice import SetVoiceBehavior

# Async Behaviors
from eb_behaviors.lights import LightsBehavior

#import eb_behaviors.async_behaviors

# Constants
DEFAULT_RESOURCE_DIR = '/home/system/catkin_robot/resources/'



behavior_interrupt = False

def signal_handler(signal, frame):
    global behavior_interrupt
    behavior_interrupt = True

def interrupt_check():
    global behavior_interrupt
    #print("INTERRUPT_CHECK Called")
    if behavior_interrupt:
        #print("RETURNING INTERRUPT TRUE")
        return True
    else:
        #print("RETURNING INTERRUPT FALSE")
        return False

    
# ==============================================================================
class BehaviorUtils():
    def __init__(self):
        self.status_pub = rospy.Publisher('/system_status', SystemStatus, queue_size=8)

        # Initialize pygame for async music playback
        pygame.init()
        #pygame.mixer.init() #Initialzing pyamge mixer

        # Behavior sounds are stored in the shared behavior resource directory
        self.resource_dir = rospy.get_param('resource_dir', DEFAULT_RESOURCE_DIR)
        # OLD: '/home/system/catkin_robot/src/eb/eb_behaviors/resources')
        self.music_dir = os.path.join(self.resource_dir, 'sounds/music')
        self.sound_bites_dir = os.path.join(self.resource_dir, 'sounds/sound_bites')
        self.scripts_dir = os.path.join(self.resource_dir, 'csv_scripts')

        if True:   # disable this for testing without speech enabled
            # Start up speech server for all behaviors to use
            rospy.loginfo("Behavior Server: Waiting for speech server...")
            self.speech_client = actionlib.SimpleActionClient("/speech_service", robot_voice.msg.speechAction)
            self.speech_client.wait_for_server()
            rospy.loginfo("Behavior Server: Speech server ready.")
        
    def send_status_update(self, item, status):
        # Send status update to system monitor
        status_msg = SystemStatus()
        status_msg.item = item
        status_msg.status = status
        self.status_pub.publish(status_msg)


    def speak(self, text_to_speak, wait_for_result = True):
        if self.speech_client:
            speech_goal = robot_voice.msg.speechGoal(text_to_speak)
            rospy.loginfo("Behavior Server: Speaking...[%s]" % (text_to_speak))
            self.speech_client.send_goal(speech_goal)
            if wait_for_result:
                result = self.speech_client.wait_for_result() # wait for speech to complete
                rospy.loginfo("Speech goal returned result: %d" % (result))
            else:
                rospy.loginfo("Speech not waiting for result." )

    def play_sound(self, audio_type, audio_name):
        # utility for playing music or sound bites.
        # for music we use pygame, because playsound can't be interrupted / stopped.
        # Note that Dance does something different, due to the beat detector
        rospy.loginfo('BehaviorUtils:play_sound: Playing Audio [%s]...' % (audio_name))
        if audio_type == 'music':
            self.audio_path = os.path.join(self.music_dir, audio_name)
        elif audio_type == 'sound_bite':
            self.audio_path = os.path.join(self.sound_bites_dir, audio_name)
        else:
            rospy.logwarn('BehaviorUtils:play_sound: bad audio_type: [%s]' % (audio_type))
                
        #audio_name = 'walking_on_sunshine_full.wav'
        #audio_name = 'walle_name.wav'

        # Load music file       
        # If you get an error, check file permissions of the audio file!
        # Also, some files from Mac don't work. Use audacity to convert to new file
        rospy.loginfo("BehaviorUtils:play_sound:    Loading: %s"% (self.audio_path))
        #playsound(self.audio_path) # Not used. Does not allow for interrupting audio
        pygame.mixer.music.load(self.audio_path)
        rospy.loginfo("BehaviorUtils:play_sound:    Audio loaded. Playing... ")
        # Start playing the audio!
        pygame.mixer.music.play(0) # non-blocking

        
# ==============================================================================
class BehaviorServer():
    def __init__(self):
        rospy.init_node('behavior_server')
        self.log_name = 'behavior_server'
        self.speech_client = None
        self.new_command = ''
        self.new_param1 = ''
        self.new_param2 = ''
        self.EAR_CMD_AI_MODE       = 2 # Chasing (thinking)
        self.EAR_CMD_BEHAVIOR_MODE = 3 # Rainbow
        self.behavior_utils = BehaviorUtils()
        self.speak = self.behavior_utils.speak
        self.current_running_behavior = ''
        self.ai_enabled = True        

        # Get behavior server parameters. These can be set in the master launch file
        self.robot_startup_behavior = rospy.get_param('~robot_startup_behavior', 'WAKE') # WAKE or NOTHING
        rospy.loginfo('Behavior Server Param: ~robot_startup_behavior = %s', self.robot_startup_behavior)
        self.robot_idle_behavior = rospy.get_param('~robot_idle_behavior', 'IDLE') # IDLE or NOTHING
        rospy.loginfo('Behavior Server Param: ~robot_idle_behavior = %s', self.robot_idle_behavior)

        if True:   # disable this for testing without speech enabled
            # Start up speech server for all behaviors to use
            rospy.loginfo("Behavior Server: Waiting for speech server...")
            self.speech_client = actionlib.SimpleActionClient("/speech_service", robot_voice.msg.speechAction)
            self.speech_client.wait_for_server()
            rospy.loginfo("Behavior Server: Speech server ready.")

        # PUBLISHERS
        self.head_peek_pub = rospy.Publisher('/head_peek',   Bool, queue_size=1)
        
        # Tell AI to wait when behavior starts and continue when behavior completes
        # (Note that behaviors can come from other sources besides the AI, such as bluetooth phone)
        self.behavior_wait_pub = rospy.Publisher('/behavior/wait', Bool, queue_size=2) 
        self.behavior_complete_pub = rospy.Publisher('/behavior/complete', Bool, queue_size=2)
        self.speech_reco_mode_pub =  rospy.Publisher('/speech_recognition/mode', String, queue_size=10)

        # For enabling/disabling conversational AI
        self.ai_enable_pub = rospy.Publisher('/ai_enable',   Bool, queue_size=4)

        # initialize all behaviors
        self.wake_behavior =        WakeBehavior(      self.behavior_utils, interrupt_check, self.ai_is_enabled)
        self.sleep_behavior =       SleepBehavior(     self.behavior_utils, interrupt_check)
        self.example_behavior =     ExampleBehavior(   self.behavior_utils, interrupt_check)
        self.nothing_behavior =     NothingBehavior(   self.behavior_utils, interrupt_check)
        self.idle_behavior =        IdleBehavior(      self.behavior_utils, interrupt_check)
        self.turn_behavior =        TurnBehavior(      self.behavior_utils, interrupt_check)
        self.move_behavior =        MoveBehavior(      self.behavior_utils, interrupt_check)
        self.pose_behavior =        PoseBehavior(      self.behavior_utils, interrupt_check)
        self.head_center_behavior = HeadCenterBehavior(self.behavior_utils, interrupt_check)
        self.happy_behavior =       HappyBehavior(     self.behavior_utils, interrupt_check)
        self.dance_behavior =       DanceBehavior(     self.behavior_utils, interrupt_check)
        self.intro_behavior =       IntroBehavior(     self.behavior_utils, interrupt_check)
        self.joke_behavior =        JokeBehavior(      self.behavior_utils, interrupt_check)
        self.bad_robot_behavior =   BadBehavior(       self.behavior_utils, interrupt_check)
        self.set_voice_behavior =   SetVoiceBehavior(  self.behavior_utils, interrupt_check)
        self.lights_behavior =      LightsBehavior(    self.behavior_utils, interrupt_check)


        # SUBSCRIBERS
        behavior_cmd_sub = rospy.Subscriber('/behavior/cmd', CommandState, self.behavior_command_cb)
        touch_sub = rospy.Subscriber('/head_touch', Int16, self.head_touch_cb)
        
        rospy.loginfo("Behavior Server: init complete.")

    def ai_is_enabled(self):
        return self.ai_enabled

        
    def head_touch_cb(self, data):
        global behavior_interrupt
        # data is ignored. Whenever the head detects a touch it sends this message
        # for now, we just do a happy dance. May change this later to be handled inside behaviors.

        
        if self.current_running_behavior == 'SLEEP':
            rospy.loginfo("Behavior Server: ===================================================================")
            rospy.loginfo("Behavior Server: =======> IGNORING HAPPY COMMAND (head touch?) while in SLEEP MODE!")

        elif self.current_running_behavior == 'IDLE':
            # Do a happy dance, but only if currently idle
            self.new_command = "HAPPY"
            self.new_param1 = ''
            self.new_param2 = ''
            behavior_interrupt = True # Tell idle behavior to exit
        
        else:        
            #Running some other behavior. When user touches head, just exit the behavior.
            self.new_command = ''
            self.new_param1 = ''
            self.new_param2 = ''
            behavior_interrupt = True # Tell any currently running behavior to exit
            print()
            rospy.loginfo("**************************************************************************")
            rospy.loginfo("Behavior Server: HAPPY (HEAD TOUCH) Just Interrupting current behavior (if any)")
            rospy.loginfo("**************************************************************************")
        

    # ===============================================================================================        
    def behavior_command_cb(self, data):
        print("Behavior Server:behavior_command_cb Got behavior command:", data)        

        command = data.commandState.upper()
        param1 = data.param1.upper()
        param2 = data.param2.upper()

        print("")
        rospy.loginfo("*********************************************")
        rospy.loginfo("Behavior Server: Got behavior command: %s " % command)        
        rospy.loginfo("                 Param1: %s, Param2: %s" % (param1, param2))        
        rospy.loginfo("*********************************************")

        global behavior_interrupt

        # process ASYNC commands that can run concurrently with a running behavior
        #if command == "BOW":
        # Use this to trigger Head Peek behavior *while in sleep mode!*
        # self.head_peek_pub.publish(True) # If TRUE, move head up for a bit

        # Check for Sleep Mode
        if self.current_running_behavior == 'SLEEP':
            if command != "WAKE" and command != "WAKEUP":
                rospy.loginfo("Behavior Server: ===================================================================")
                rospy.loginfo("Behavior Server: =======> IGNORING BEHAVIOR COMMAND %s while in SLEEP MODE!" % (command))
                return

        if command == "AI_MODE":
            self.handle_ai_mode_behavior(param1, param2)
            return

        elif command == "NAME_ONLY":
            self.handle_name_only_behavior(param1, param2)
            return

        elif command == "SAY":
            # Command to say a phrase, like "Hello". Just say it without interrupting current behavior
            self.speak(param1, False) # don't block waiting for it to finish speaking
            return

        elif command == "CONNECT":
            # comamnd from bluetooth phone to connect to wifi. Just ignore
            command == ""
            return


        # Done with special case commands. Process normal behaviors:
        else: # Tell behavior system to process this new command
            self.new_command = command
            self.new_param1 = param1
            self.new_param2 = param2
            behavior_interrupt = True # Tell any currently running behavior to exit
            #print("Behavior Server interrupting current behavior (if any)")
                           

        print("Behavior Server:behavior_command_cb done")        
    # ===============================================================================================        


    def handle_name_only_behavior(self, param1, param2):  # TODO TODO TODO             
        # If user says robot's name, turn AI back on, but ONLY if it's NOT in SLEEP MODE!
        # does not interrupt current behavior
        rospy.loginfo('%s:handle_name_only_behavior: Executing behavior' % (self.log_name))
        rospy.loginfo( "Param1: '%s'", param1)
        rospy.loginfo( "Param2: '%s'", param2)

        if self.current_running_behavior == 'SLEEP':
            rospy.loginfo("Behavior Server: ===================================================================")
            rospy.loginfo("Behavior Server: =======> IGNORING ROBOT NAME ONLY while in SLEEP MODE!")
        else:        
            rospy.loginfo("Behavior Server: ===================================================================")
            rospy.loginfo("Behavior Server: =======> Got ROBOT NAME with no command. Un-Pausing AI!")
            ## self.pub_ear_mode.publish(self.EAR_CMD_AI_MODE) # Indicate AI mode 
            ## self.ai_enable_pub.publish(True)  
            ## self.speak('I am listening')  # DISABLE handled in AI
            self.speech_reco_mode_pub.publish('stream') # Return to streaming mode
            self.behavior_wait_pub.publish(False) # AI does not need to wait any longer


    def set_ai_enabled(self, enable):
        if enable:
            rospy.loginfo('%s:set_ai_enabled: Enabling AI mode' % (self.log_name))
            self.ai_enabled = True
        else:
            rospy.loginfo('%s:set_ai_enabled: Disabling AI mode.' % (self.log_name))
            self.ai_enabled = False

        if self.current_running_behavior == 'SLEEP':
            rospy.loginfo("Behavior Server: ===================================================================")
            rospy.loginfo("Behavior Server: =======> AI_MODE deferred while in SLEEP MODE.")
        else:
            if self.ai_enabled:
                self.ai_enable_pub.publish(True)  # Enable AI
            else:
                self.ai_enable_pub.publish(False) # Disable AI        
 
 
    def handle_ai_mode_behavior(self, param1, param2):
        # does not interrupt current behavior
        # used to turn AI off and keep it there (from phone, etc) until re-enabled
        rospy.loginfo('%s:handle_ai_mode_behavior: Executing behavior' % (self.log_name))
        rospy.loginfo( "Param1: '%s'", param1)
        rospy.loginfo( "Param2: '%s'", param2)

        if param1 == "TRUE":
            self.set_ai_enabled(True)

        elif param1 == "FALSE":
            self.set_ai_enabled(False)

        elif param1 == "TOGGLE":
            if not self.ai_enabled:
                self.set_ai_enabled(True)
            else:                
                self.set_ai_enabled(False)

        elif param1 == "PAUSE_TALKING": 
            # put in keyword mode, but only until the next behavior command comes in
            # used when user says no to AI "do you want to talk?" Cleared if user says the keyword
            # does not change self.ai_enabled state

            if self.current_running_behavior != 'SLEEP':
                rospy.loginfo('%s:handle_ai_mode_behavior: AI mode: PAUSE_TALKING Mode now active.' % (self.log_name))
                self.speech_reco_mode_pub.publish('keyword') # Listen for keywords only        

        else:
            rospy.logwarn('%s:handle_ai_mode_behavior: Bad AI mode parameter!  Ignored command!' % (self.log_name))



    def run(self):
        global behavior_interrupt
        
        # Set startup behavior
        self.new_command = self.robot_startup_behavior
        self.new_param1 = ''
        self.new_param2 = ''
        
        # run until killed
        while not rospy.is_shutdown(): 

            if self.new_command == '' or self.new_command == 'STOP':
                # no command to execute. Do Idle funtion
                # 'STOP' just cancels whatever was running
                self.new_command = self.robot_idle_behavior
                self.new_param1 = ''
                self.new_param2 = ''

            # Process new command
            command = self.new_command
            param1 = self.new_param1
            param2 = self.new_param2
            self.new_command = ''
            self.new_param1 = ''
            self.new_param2 = ''
            #print("server RUN setting INTERRUPT to FALSE")
            behavior_interrupt = False # clear interrupt before launching the new behavior

            self.behavior_utils.send_status_update('BEHAVIOR', command)
            self.current_running_behavior = command
           
            if command == "IDLE" or command == "NOTHING":
                # If enabled, Tell AI that we are done processing the prior behavior, 
                # so it can continue having conversations
                self.behavior_wait_pub.publish(False) # AI does not need to wait any longer
                ## self.pub_ear_mode.publish(self.EAR_CMD_AI_MODE) # Indicate AI mode
            else:
                # In case the command came from somewhere other than the AI (eg., phone),
                # Tell AI to wait until behavior completes
                self.behavior_wait_pub.publish(True)
             
            # Run the selected behavior
            if command == "SLEEP":
                self.sleep_behavior.execute_behavior(param1, param2)
                # spins until interrupted to prevent IDLE from running                
                
            elif command == "IDLE":
                self.idle_behavior.execute_behavior(param1, param2)
                # spins until interrupted, doing face tracking, random movements, etc.

            elif command == "NOTHING":
                self.NothingBehavior.execute_behavior(param1, param2)
                # spins until interrupted, doing nothing.

            elif command == "WAKEUP" or command == "WAKE":
                self.wake_behavior.execute_behavior(param1, param2)
                # returns when finished or interrupted
                            
            elif command == "HEAD_CENTER":
                self.head_center_behavior.execute_behavior(param1, param2)
                # returns when finished or interrupted
            
            elif command == "EXAMPLE":
                self.example_behavior.execute_behavior(param1, param2)
                # returns when finished or interrupted

            elif command == "TURN":
                self.turn_behavior.execute_behavior(param1, param2)
                # returns when finished or interrupted

            elif command == "MOVE":
                self.move_behavior.execute_behavior(param1, param2)
                # returns when finished or interrupted

            elif command == "POSE":
                self.pose_behavior.execute_behavior(param1, param2)
                # returns when finished or interrupted

            elif command == "HAPPY":
                self.happy_behavior.execute_behavior(param1, param2)
                # returns when finished or interrupted

            elif command == "DANCE" or command == "SONG":
                self.dance_behavior.execute_behavior(param1, param2)
                # returns when finished or interrupted

            elif command == "INTRO":
                self.intro_behavior.execute_behavior(param1, param2)
                # returns when finished or interrupted
                
            elif command == "JOKE" or command == "TELL_JOKE" or command == "TELL_A_JOKE":
                self.joke_behavior.execute_behavior(param1, param2)
                # returns when finished or interrupted

            elif command == "LIGHTS" or command == "TELL_JOKE" or command == "TELL_A_JOKE":
                self.lights_behavior.execute_behavior(param1, param2)
                # returns when finished or interrupted

            elif command == "BAD": # Do "bad robot" behavior
                self.bad_robot_behavior.execute_behavior(param1, param2)
                # returns when finished or interrupted

            elif command == "BOW": # KLUDGE - do set voice behavior
                self.set_voice_behavior.execute_behavior(param1, param2)
                # returns when finished or interrupted

                
            else:
                rospy.logwarn("*********************************************")
                rospy.logwarn("Behavior Server: unknown command: [%s] Ignored" % command)
                rospy.logwarn("*********************************************")
                print("")
                    
            rospy.loginfo("*********************************************")
            rospy.loginfo("Behavior Server: behavior %s complete" % command )
            rospy.loginfo("*********************************************")
            print("")



        
if __name__=='__main__':
    # capture SIGINT signal, e.g., Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    node = BehaviorServer()
    node.run()       
        


