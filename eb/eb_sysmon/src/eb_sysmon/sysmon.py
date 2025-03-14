#!/usr/bin/env python3

import rospy
import logging
import time
import math

from system_status_msgs.msg import SystemStatus
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Int32
#from std_msgs.msg import Bool
#from geometry_msgs.msg import Point32
from std_msgs.msg import Float32
from std_msgs.msg import String

from eb_servos.set_pose import *    # for RobotPose
from eb_servos.set_servo_torque import *

import actionlib
import actionlib.action_client
from behavior_msgs.msg import CommandState 

from gui_layout import layout
#import PySimpleGUI as psg
import FreeSimpleGUI as psg

# Global Constants
EAR_CMD_OFF               = 0  # Off
EAR_CMD_MODE_1            = 1  # slow blue fade on/off
EAR_CMD_AI_ENGAGED        = 2  # Spin in white (thinking)
EAR_CMD_AI_BEHAVIOR_WAIT  = 3  # Rainbow
EAR_CMD_AI_READY          = 4  # Spin in blue



# LED RGB Colors for self.set_body_status_led()
LED_OFF =           0x000000
LED_RED =           0x1f0000
LED_GREEN =         0x000500
LED_BLUE =          0x000007
LED_YELLOW =        0x070700
LED_BRIGHT_YELLOW = 0x1f1f00
LED_PURPLE =        0x040006
LED_WHITE =         0x101010

# LED numbers.  0 = top LED. Bottom LEDS are easier to see
# LED 0-3 reserved for battery
LED_OBJECT =         4
LED_FACE_TRACKER =   5
LED_AI_STATE =       6
LED_MICROPHONE =     7
 
 
 
 
 
       
            
# ======================================================================================================

class SysMonGui():
    def __init__(self):
        rospy.init_node('eb_sysmongui')
        #rospy.on_shutdown(self.cleanup)
        self.module_name = "eb_sysmongui"
        print("*************** Starting SysMonGui **************")

        self.robotpose = RobotPose("dashboard")
        self.window = None
        self.warning_blink = False
        self.blink_on = False
        self.status_leds_enabled = False 
        self.status_leds_initialized = False      
        self.battery_reported_level = 100 # Only report once when voltage drops below level
        self.last_ear_command = -1
        
        self.ai_state = '---'
        self.voice_state = '---'
        self.speech_reco_state = '---'
        self.speech_reco_text = '---'
        self.behavior_mode = '---'
        self.ai_gpt_time = '---'
        self.ai_name = '---'
        self.depth_camera_status = '---'
        self.face_detector_status = '---'
        self.face_tracker_status = '---'
        self.servo_status = '---' 
        self.body_light_mode = '---'       
        self.object_front = '---'
        self.object_rear = '---'
        self.bluetooth_phone_status = '---'

        self.battery_voltage = 0
        self.battery_update = False
        self.battery_voltage_str = "0.0 Volts"
        self.battery_text_color = "white"

        # Led 4-7 blue ("no battery") until detected)
        self.status_led_array = [0x00,0x00,0x00,0x00, 0x07,0x07,0x07,0x07] 

        # PUBLISHERS
        self.behavior_cmd_pub = rospy.Publisher('behavior/cmd', CommandState, queue_size=10)
        self.pub_ear_cmd =   rospy.Publisher('/head/ear_cmd',   UInt16, queue_size=10)        
        self.pub_eye_cmd =   rospy.Publisher('/head/eye_cmd',   UInt16, queue_size=10)        
        self.pub_eye_color = rospy.Publisher('/head/eye_color', UInt32, queue_size=10)        
        self.pub_battery_msg = rospy.Publisher('/status/speak', String, queue_size=2)        

        self.pub_body_strip_color = rospy.Publisher('/body/strip_color', UInt32, queue_size=8)
        self.pub_body_strip_mode = rospy.Publisher('/body/strip_mode', UInt16, queue_size=8)
        
        # send select status updates back to bluetooth phone
        # Don't send to often, or it overloads the phone
        self.phone_pub = rospy.Publisher('/phone_update', SystemStatus, queue_size=6)


         # Create Window
        self.battery_background_color = psg.theme_background_color()
        psg.set_options(font=('Arial Bold', 12))
        self.window = psg.Window('EB Robot Dashboard', layout, size=(450, 750), 
            element_justification='c', finalize=True, enable_close_attempted_event=True)


        # SUBSCRIBERS - enable after window created!
        self.status_update_sub = rospy.Subscriber("/system_status", SystemStatus, self.status_update_callback)
        self.battery_sub = rospy.Subscriber("/battery_voltage", Float32, self.battery_callback)
        

        rospy.loginfo("%s: Initialization complete." % (self.module_name))     

    #------------------------------------ Init end --------------------------------------        



    def send_behavior_command(self, command, param1='', param2=''):
        print("eb_sysmon: sending behavior command: %s parm1: %s parm2: %s" % (command, param1, param2))

        msg = CommandState()
        msg.commandState = command
        msg.param1 = param1
        msg.param2 = param2
        self.behavior_cmd_pub.publish(msg)

        print("eb_sysmon: done.")

    def send_ear_command(self, command):
        # Don't SPAM the arduino with multiple requests
        if command != self.last_ear_command:
            rospy.loginfo("%s:Sending Ear Command [%d]" % (self.module_name, command))
            self.pub_ear_cmd.publish(command)
            self.last_ear_command = command


    def set_body_status_led(self, led_number, color, override=False):
        # Override can be used if robot is in sleep mode to light an LED
        #print("eb_sysmon:DBG: set_body_status_led called.")
        if led_number > 7:    
            rospy.logwarn("%s:set_body_status_led: bad LED number [%d]" % 
                (self.module_name, led_number))

        else:
            if not override:
                self.status_led_array[led_number] = color

            if self.status_leds_enabled or override:
                #print("eb_sysmon:DBG: led_number        : %d" % led_number)
                led_command = 1 << (led_number + 24)
                #print("eb_sysmon:DBG: shifted led_number: %08x" % led_command)
                led_command = led_command + color
                #print("eb_sysmon:DBG: with color:         %08x" % led_command)
                #print("")
                self.pub_body_strip_color.publish(led_command)


    def enable_body_status_leds(self):
        # Enable and set all leds to current config
        # print("eb_sysmon:DBG:Enabling LEDs from Array")
        self.status_leds_enabled = True

        for led_number, color in enumerate(self.status_led_array):
            #print("eb_sysmon:DBG: led_number        : %d" % led_number)
            led_command = 1 << (led_number + 24)
            #print("eb_sysmon:DBG: shifted led_number: %08x" % led_command)
            led_command = led_command + color
            #print("eb_sysmon:DBG: with color:         %08x" % led_command)
            #print("")
            self.pub_body_strip_color.publish(led_command)

        #print("eb_sysmon:DBG:Done Enabling LEDs from Array")
         

    def set_battery_leds(self, led1, led2, led3, led4):
        self.set_body_status_led(0, led1) 
        self.set_body_status_led(1, led2) 
        self.set_body_status_led(2, led3) 
        self.set_body_status_led(3, led4) 
    

    def battery_callback(self, data):

        # Also lights top 4 body LEDs to indicate voltage level

        self.battery_voltage = data.data
        volts_str = "%2.1f" % self.battery_voltage
        #self.battery_voltage_str = "%2.2f Volts" % self.battery_voltage
        self.battery_voltage_str = volts_str + " Volts"
        self.battery_background_color = "green"
        self.battery_text_color = "white"
        
        if self.battery_voltage < 10.00:   # No battery 
            self.battery_voltage_str = "No Battery"
            self.battery_background_color = psg.theme_background_color() # "light gray"
            self.set_battery_leds(LED_BLUE, LED_BLUE, LED_BLUE, LED_BLUE) 
            self.battery_reported_level = 0 # prevent any warnings
            
        elif self.battery_voltage <= 12.5:   # 2 min 
            self.battery_voltage_str = volts_str + "v 2min!"
            self.battery_background_color = "red" 
            self.set_battery_leds(LED_RED, LED_RED, LED_RED, LED_RED) 
            if self.battery_reported_level > 2: # min
                rospy.logwarn("Wheel_Control: BATTERY VOLTAGE CRITICAL! < 2 min")                    
                self.pub_battery_msg.publish(
                    "Warning, Warning, My battery is critical, Less than 2 minutes to shutdown!")
                self.battery_reported_level = 2

        elif self.battery_voltage <= 14.0:   # 5 min 
            self.battery_voltage_str = volts_str + "v <5min"
            self.battery_background_color = "red"
            self.set_battery_leds(LED_RED, LED_RED, LED_BRIGHT_YELLOW, LED_BRIGHT_YELLOW) 
            if self.battery_reported_level > 5:
                rospy.logwarn("Wheel_Control: BATTERY VOLTAGE CRITICAL! < 5 min")                    
                self.pub_battery_msg.publish("Warning, My battery is critical, 5 minutes to shutdown.")
                self.battery_reported_level = 5

        elif self.battery_voltage <= 14.25:    # 10 min
            self.battery_voltage_str = volts_str + "v <10min"
            self.battery_background_color = "orange"
            self.battery_text_color = "black"
            self.set_battery_leds(
                LED_BRIGHT_YELLOW, LED_BRIGHT_YELLOW, LED_BRIGHT_YELLOW, LED_BRIGHT_YELLOW) 
            if self.battery_reported_level > 10:
                self.pub_battery_msg.publish("Please tell Dave that my battery is low. I have only 10 minutes remaining.")
                self.battery_reported_level = 10

        elif self.battery_voltage <= 14.5:    # 20 min
            self.battery_voltage_str = volts_str + "v <20min"
            self.battery_background_color = "orange"
            self.battery_text_color = "black"
            self.set_battery_leds(LED_YELLOW, LED_YELLOW, LED_YELLOW, LED_GREEN) 
            if self.battery_reported_level > 20:
                self.pub_battery_msg.publish("By the way, I only have 20 minutes left on my battery.")
                self.battery_reported_level = 20
             
        elif self.battery_voltage <= 15.0:    # 30 min
            self.battery_voltage_str = volts_str + "v <30min"
            self.battery_background_color = "yellow"
            self.battery_text_color = "black"
            self.set_battery_leds(LED_YELLOW, LED_YELLOW, LED_GREEN, LED_GREEN) 
             
        elif self.battery_voltage <= 15.25:    # 60 min
            self.battery_voltage_str = volts_str + "v <60min"
            self.battery_background_color = "blue"
            self.set_battery_leds(LED_YELLOW, LED_GREEN, LED_GREEN, LED_GREEN) 
            
        else:
            self.set_battery_leds(LED_GREEN, LED_GREEN, LED_GREEN, LED_GREEN) 
            self.battery_reported_level = 100 # reset on good battery inserted  
             

        self.battery_update = True
     

    def status_update_callback(self, status_msg):
        # Gets updates from modules and displays them. (sent via topic "/system_status")

        item_str = status_msg.item
        status_str = status_msg.status
        # rospy.loginfo("%s: Got Status update [%s] [%s]" % (self.module_name, item_str, status_str))            


        # SPECIAL CASES
        
        if item_str == 'BODY_LIGHT_MODE': 
            self.body_light_mode = status_str
            
            if status_str == 'OFF' or status_str == 'BEHAVIOR':
                # Turn lights off (behavior might turn them on)
                self.status_leds_enabled = False
                self.pub_body_strip_color.publish(0xff000000) # force all LEDS off
                rospy.loginfo("%s: BODY_LIGHT_MODE: turning leds off" % (self.module_name))

                
            elif status_str == 'ON' or status_str == 'STATUS':
                # Turn status lights on and update
                self.status_leds_enabled = True
                self.enable_body_status_leds()
                rospy.loginfo("%s: status_leds_enabled" % (self.module_name))


        elif item_str == 'SPEECH_RECO_STATE': # speech recognition server state
            #print("********************* SPEECH_RECO_STATE *****************")
            self.speech_reco_state = status_str 
            
            # Special LED case for keyword in sleep mode
            if not self.status_leds_enabled:
                # sleep mode or behavior running          
                if self.speech_reco_state == "KEYWORD_DETECTED":
                    #print("********************* SPEECH_RECO_STATE KEYWORD_DETECTED *****************")
                    rospy.loginfo("%s: KEYWORD_DETECTED in SLEEP MODE: mic led WHITE" % (self.module_name)) 
                    self.set_body_status_led(LED_MICROPHONE, LED_WHITE, True) # with Override
                elif self.speech_reco_state == "KEYWORD_RUNNING":
                    rospy.loginfo("%s: KEYWORD_RUNNING in SLEEP MODE: mic led OFF" % (self.module_name)) 
                    self.set_body_status_led(LED_MICROPHONE, LED_OFF, True) # with Override




        elif item_str == 'AI_STATE': # AI state machine
            self.ai_state = status_str

            # Indicate state by ear color
            # Future option, mess with color? rostopic pub -1 /head/ear_color std_msgs/UInt32 "0x002f00" (green)
            if self.status_leds_enabled:
                if self.ai_state == "READY": # Ready, but not tracking
                    self.send_ear_command(EAR_CMD_AI_READY) #  Spinning blue

                elif self.ai_state == "DISABLED":
                    self.send_ear_command(EAR_CMD_OFF) # Off

                elif self.ai_state == "BEHAVIOR_WAIT":
                    self.send_ear_command(EAR_CMD_AI_BEHAVIOR_WAIT)  # Rainbow
                    
                else:
                    self.send_ear_command(EAR_CMD_AI_ENGAGED) # Spinning white



        # NORMAL UPDATES
            
        elif item_str == 'DEPTH_CAMERA':
            self.depth_camera_status = status_str

        elif item_str == 'FACE_DETECTOR':
            self.face_detector_status = status_str
            
        elif item_str == 'FACE_TRACKER':
            self.face_tracker_status = status_str

        elif item_str == 'OBJECT_FRONT': # WARN if blocked, FAIL if no sensor
            self.object_front = status_str

        elif item_str == 'OBJECT_REAR': # WARN if blocked, FAIL if no sensor
            self.object_rear = status_str

        elif item_str == 'VOICE':
            self.voice_state = status_str

        elif item_str == 'AI_GPT_TIME': # 
            self.ai_gpt_time = status_str  
            # PHONE UPDATE - send this status to bluetooth phone
            self.phone_pub.publish(status_msg)

        elif item_str == 'AI_NAME': # Name of person recognized 
            self.ai_name = status_str  

        elif item_str == 'BEHAVIOR': # state machine
            self.behavior_mode = status_str   # currently running behavior
            # PHONE UPDATE - send this status to bluetooth phone
            self.phone_pub.publish(status_msg)

        elif item_str == 'BLUETOOTH_PHONE': # from body arduino
            self.bluetooth_phone_status = status_str  

        elif item_str == 'SPEECH_RECO_TEXT': # speech recognition text as it comes in
            self.speech_reco_text = status_str  

            

        else:
            rospy.logwarn("%s: Got status for UNKNOWN ITEM: [%s] [%s]" % (self.module_name, item_str, status_str))

     
 

#===============================================================================
#                                        LOOP
#===============================================================================

    def run(self):

        rospy.sleep(0.1)   
        # for some weird reason, this fails when attempted during init.
        # so, doing it here. 
        rospy.loginfo("%s: Turning off body LEDs." % (self.module_name))
        #self.pub_body_strip_mode.publish(1) # set status leds to normal mode on startup            
        self.pub_body_strip_color.publish(0xff000000) # Turn off all status leds on startup   

    
        while not rospy.is_shutdown():

            if self.battery_update:
                self.window['-BATTERY-'].update(self.battery_voltage_str, background_color = self.battery_background_color, text_color = self.battery_text_color)
                self.battery_update = False

            ###################################################################     
            # GUI Events
               
            event, values = self.window.read(timeout=0)
            
            #if event == psg.WINDOW_CLOSE_ATTEMPTED_EVENT:
            #    pass

            if event == psg.WIN_CLOSED or event == 'Exit':
                break

            elif event == '-POSE0-':
                self.robotpose.move(0, 0.3) # Pose, Speed
                
            elif event == '-POSE1-':
                self.robotpose.move(1, 0.3) # Pose, Speed
                
            elif event == '-POSE2-':
                self.robotpose.move(2, 0.3) # Pose, Speed
                
            elif event == '-POSE3-':
                self.robotpose.move(3, 0.3) # Pose, Speed
                
            elif event == '-POSE4-':
                self.robotpose.move(4, 0.3) # Pose, Speed

            elif event == '-POSE5-':
                self.robotpose.move(5, 0.3) # Pose, Speed


            # Lights
            elif event == '-EYES_ON-':
                self.pub_eye_cmd.publish(2) # 2 = Turn eyes on, normal blink mode
            elif event == '-EARS_ON-':
                self.send_ear_command(EAR_CMD_AI_BEHAVIOR_WAIT) #  Turn ear lights on, rainbow mode
            elif event == '-BODY_ON-':
                ## self.pub_body_strip_mode.publish(1) # 1 = Normal mode
                # Turn status lights on and update
                self.enable_body_status_leds()


            elif event == '-EYES_OFF-':
                self.pub_eye_cmd.publish(0) # Turn eyes off
            elif event == '-EARS_OFF-':
                self.send_ear_command(EAR_CMD_OFF) # Turn ears off
            elif event == '-BODY_OFF-':
                ##self.pub_body_strip_mode.publish(0) # 1 = Force body lights off
                # Turn lights off (behavior might turn them on)
                self.status_leds_enabled = False
                self.pub_body_strip_color.publish(0xff000000) # force all LEDS off


            # Behaviors
            elif event == '-HEAD_CENTER-':
                self.send_behavior_command('HEAD_CENTER', 'x', 'x')
                
            elif event == '-WAKE-':
                self.send_behavior_command('WAKE', 'x', 'x')
                
            elif event == '-SLEEP-':
                self.send_behavior_command('SLEEP', 'x', 'x')
                
            elif event == '-STOP-':
                self.send_behavior_command('STOP', 'x', 'x')
                
            elif event == '-FOLLOW-':
                self.send_behavior_command('FOLLOW_ME')
                
            elif event == '-DANGER-':
                self.send_behavior_command('DANGER')
                
            elif event == '-DANCE-':
                self.send_behavior_command('DANCE')
                
            elif event == '-HAPPY-':
                self.send_behavior_command('HAPPY')
         

            ###################################################################     
            # System Status updates

            new_color = "LightGrey"

            # AI State Machine state
            if self.ai_state == "READY":
                self.set_body_status_led(LED_AI_STATE, LED_BLUE)
                new_color = "LightGrey"
            elif self.ai_state == "DISABLED":
                self.set_body_status_led(LED_AI_STATE, LED_BRIGHT_YELLOW)
                new_color = "orange"
            elif self.ai_state == "ENGAGED":
                self.set_body_status_led(LED_AI_STATE, LED_GREEN)
                new_color = "yellow"
            elif self.ai_state == "LISTEN" or self.ai_state == "LISTEN_WAIT":
                self.set_body_status_led(LED_AI_STATE, LED_WHITE)
                new_color = "green" 
            elif self.ai_state == "BEHAVIOR_WAIT":
                self.set_body_status_led(LED_AI_STATE, LED_PURPLE)
                new_color = "green" 
            else:
                new_color = "LightGrey"
            self.window['-ai_state-'].update(
                self.ai_state, background_color = new_color)


            # Voice (Text to Speech) state
            if self.voice_state == "READY":
                new_color = "LightGrey"
            elif self.voice_state == "SPEAKING":
                new_color = "green" 
            else:
                new_color = "LightGrey"
            self.window['-voice_state-'].update(
                self.voice_state, background_color = new_color)


            # AI GPT Latency Time
            ai_gpt_time_float = 0.0
            try:
                ai_gpt_time_float = float(self.ai_gpt_time)
            except ValueError:
                pass
            if ai_gpt_time_float <=0.0: 
                new_color = "LightGrey"
            elif ai_gpt_time_float <0.800: # seconds
                new_color = "green"
            elif ai_gpt_time_float <1.000:
                new_color = "yellow"
            else: 
                new_color = "red"
            self.window['-ai_gpt_time-'].update(
                self.ai_gpt_time, background_color = new_color)


            # From AI: Name of person recognized
            if self.ai_name == "":
                new_color = "LightBlue" 
            elif self.ai_name == "RESET":
                new_color = "LightBlue"
            elif self.ai_name == "---":
                new_color = "LightGrey"
            else:
                new_color = "green" # any name
            self.window['-ai_name-'].update(
                self.ai_name, background_color = new_color)


            # Behavior Mode
            if self.behavior_mode == "IDLE":
                new_color = "green" 
            elif self.behavior_mode == "SLEEP":
                new_color = "orange"
            elif self.behavior_mode == "---":
                new_color = "LightGrey"
            else:
                new_color = "LightBlue" # all other modes
            self.window['-behavior_mode-'].update(
                self.behavior_mode, background_color = new_color)


            # Speech Recogniton State
            if self.speech_reco_state == "MIC_FAIL":
                self.set_body_status_led(LED_MICROPHONE, LED_RED) 
                new_color = "red"
            elif (self.speech_reco_state == "MIC_READY" or 
                self.speech_reco_state == "LISTENING" or
                self.speech_reco_state == "LISTEN_DONE"):
                self.set_body_status_led(LED_MICROPHONE, LED_GREEN) 
                new_color = "green"
            elif self.speech_reco_state == "KEYWORD_RUNNING":
                self.set_body_status_led(LED_MICROPHONE, LED_BLUE) 
                new_color = "LightBlue"
            elif self.speech_reco_state == "KEYWORD_DETECTED":
                self.set_body_status_led(LED_MICROPHONE, LED_WHITE) 
                new_color = "orange"
            elif self.speech_reco_state == "DISABLED":
                self.set_body_status_led(LED_MICROPHONE, LED_BRIGHT_YELLOW) 
                new_color = "orange"
            else:
                new_color = "LightGrey"
            self.window['-speech_reco_state-'].update(
                self.speech_reco_state, background_color = new_color)

                
            # Depth Camera    
            if self.depth_camera_status == "WAITING_FOR_FRAME":
                new_color = "LightGrey"
            elif self.depth_camera_status == "FAIL":
                new_color = "red"
            elif self.depth_camera_status == "RUNNING":
                new_color = "green"
            else:
                new_color = "LightGrey"
            self.window['-depth_camera_status-'].update(
                self.depth_camera_status, background_color = new_color)

            # Face Detector
            if self.face_detector_status == "LOADING_FACES":
                new_color = "LightGrey"
            elif self.face_detector_status == "WAIT_FOR_FRAME":
                new_color = "red"
            elif self.face_detector_status == "RUNNING":
                new_color = "green"
            else:
                new_color = "LightGrey"
            self.window['-face_detector_status-'].update(
                self.face_detector_status, background_color = new_color)

            # Face Tracker (Led 3)
            if self.face_tracker_status == "WAIT_FIRST_FACE":
                self.set_body_status_led(LED_FACE_TRACKER, LED_RED) # no camera yet
                new_color = "red"
            elif self.face_tracker_status == "DISABLED":
                self.set_body_status_led(LED_FACE_TRACKER, LED_YELLOW) 
                new_color = "yellow"
            elif self.face_tracker_status == "TRACKING":
                self.set_body_status_led(LED_FACE_TRACKER, LED_GREEN) 
                new_color = "green"
            elif self.face_tracker_status == "GOT_NAME":
                self.set_body_status_led(LED_FACE_TRACKER, LED_PURPLE) 
                new_color = "green"
            elif self.face_tracker_status == "READY":
                self.set_body_status_led(LED_FACE_TRACKER, LED_BLUE) 
                new_color = "LightBlue"
            else:
                new_color = "LightGrey"
            self.window['-face_tracker_status-'].update(
                self.face_tracker_status, background_color = new_color)
                
            # Collision Avoidance Front   
            if self.object_front == "STOP": 
                self.set_body_status_led(LED_OBJECT, LED_RED)
                new_color = "red"
            elif self.object_front == "SLOW":
                self.set_body_status_led(LED_OBJECT, LED_YELLOW)
                new_color = "yellow"
            elif self.object_front == "AVOID":
                self.set_body_status_led(LED_OBJECT, LED_BLUE)
                new_color = "LightBlue"
            elif self.object_front == "OK":
                self.set_body_status_led(LED_OBJECT, LED_GREEN)
                new_color = "green"
            else:
                new_color = "LightGrey"
            self.window['-object_front-'].update(
                self.object_front, background_color = new_color)

            # Collision Avoidance Rear   
            if self.object_rear == "STOP":
                self.set_body_status_led(LED_OBJECT, LED_RED)
                new_color = "red"
            elif self.object_rear == "SLOW":
                self.set_body_status_led(LED_OBJECT, LED_YELLOW)
                new_color = "yellow"
            elif self.object_rear == "OK":
                self.set_body_status_led(LED_OBJECT, LED_GREEN)
                new_color = "green"
            self.window['-object_rear-'].update(
                self.object_rear, background_color = new_color)
                
                
           # Bluetooth Phone     
            if self.bluetooth_phone_status == "DISCONNECTED":
                new_color = "LightBlue"
            elif self.bluetooth_phone_status == "CONNECTED":
                new_color = "green"
            else:
                new_color = "LightGrey"
            self.window['-bluetooth_phone_status-'].update(
                self.bluetooth_phone_status, background_color = new_color)


            # Status LEDs Enabled indicator
            if self.body_light_mode == "OFF":
                new_color = "yellow"
            if self.body_light_mode == "'BEHAVIOR'":
                new_color = "LightBlue"
            elif self.body_light_mode == "STATUS":
                new_color = "green"
            else:
                new_color = "LightGrey"
            self.window['-body_light_mode-'].update(
                self.body_light_mode, background_color = new_color)

            
           # Streaming Speech recognition text     
            if (self.speech_reco_text != '' and self.speech_reco_text != '---'
                and self.voice_state != "SPEAKING"): 
                new_color = "LightBlue"
            else:
                new_color = "LightGrey"
            self.window['-speech_reco_text-'].update(
                self.speech_reco_text, background_color = new_color)



            rospy.sleep(0.1)   
                
        self.window.close()
        
        
if __name__ == '__main__':

    node = SysMonGui()
    node.run()

        
        
        
        
        
        
        

