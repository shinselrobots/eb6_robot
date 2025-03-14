#! /usr/bin/env python
# Plays back music and robot dances to the beat (and lights displays)
# Beat detction based upon example code from https://github.com/Remy2701/beat_detection/tree/main

import rospy
import time
import math
import random
import numpy as np
import sys
import os.path

# For music playback and beat detection
# we used to use pygame, because playsound can't be interrupted / stopped.
# but now we use simpleaudio instead, to get beats and allow stopping with play_obj.stop() 
import librosa
import simpleaudio
#import time
import eb_behaviors.audio_utils.audio_beat as beat

from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import String
from simple_move_msgs.msg import SimpleMove
from geometry_msgs.msg import Point, Point32


# for servos
#from eb_servos.head_servo_publishers import *
#from eb_servos.right_arm_servo_publishers import *
#from eb_servos.left_arm_servo_publishers import *

from eb_servos.set_pose import *
#from eb_servos.standard_servo_positions import *
from eb_servos.set_servo_speed import *
#from eb_servos.set_servo_torque import *

# ROS Angles package
#includes: fabs, shortest_angular_distance, normalize_angle_positive, etc.
from angles import * # Python dir() to see list


DEBUG_LIGHTS_ONLY = False 
DEBUG_PRINT_BEATS = False


song_list = [

    { "keywords": "walking on sunshine", "filename": "walking_on_sunshine_trim",
        "bpm": 110,    "dance_offset": 4 },

    { "keywords": "happy", "filename": "happy_trim",
        "bpm": 160,    "dance_offset": 4 },

    { "keywords": "im a believer", "filename": "im_a_believer_trim", 
        "bpm": 165,    "dance_offset": 4 },

    { "keywords": "white wedding", "filename": "white_wedding_trim", 
        "bpm": 147,    "dance_offset": 4 },

    { "keywords": "kryptonite", "filename": "kryptonite_trim",
        "bpm": 99,    "dance_offset": 8 },

    { "keywords": "mister roboto", "filename": "mr_roboto_trim",
        "bpm": 144,    "dance_offset": 4 },

    { "keywords": "black horse and cherry tree", "filename": "black_horse",  
        "bpm": 106,   "dance_offset": 4 },


            ]

DEFAULT_SONG_INDEX = 0
DEFAULT_FILE_NAME = 'walking_on_sunshine_trim'


class DanceBehavior():
    # Make dance movements, such as leaning side to side

    def __init__(self, behavior_utils, interrupt_check):
        self.module_name = 'dance_behavior'
        rospy.loginfo('%s: Initializing...' % (self.module_name))

        self.interrupt_check = interrupt_check
        self.send_status_update = behavior_utils.send_status_update
        self.speak = behavior_utils.speak
        self.music_dir = behavior_utils.music_dir
        self.robotpose = RobotPose(self.module_name)

        self.start_thigh_lift = 0.0
        self.start_knee_bend = 0.0
        self.start_ankle_rotate = 0.0
        self.EAR_CMD_AI_MODE       = 2 # Chasing (thinking)
        self.EAR_CMD_KEYWORD_MODE  = 3 # Rainbow
        self.ANTENNA_CENTER = -0.4
        self.imu_yaw = 0.0
        self.last_angle_rad = 0.0
        
        self.slider01 = 0.0
        self.slider02 = 0.0
        self.slider03 = 0.0

        # Subscribe to debug_sliders for tuning parameters
        slider01_sub  = rospy.Subscriber('/slider/value1',  Float32, self.slider01_cb)
        slider02_sub  = rospy.Subscriber('/slider/value2',  Float32, self.slider02_cb)
        slider03_sub  = rospy.Subscriber('/slider/value3',  Float32, self.slider03_cb)

        # get imu updates from Arduino BNO086
        imu_sub = rospy.Subscriber("/imu_orientation", Point32, self.imu_callback)
 
        # For Beat detector
        self.y = None  # librosa audio time series
        self.sr = None # sampling rate of y
        self.play_obj = None # simpleaudio play object

        # Publishers
        self.ai_enable_pub = rospy.Publisher('/ai_enable',   Bool, queue_size=2)
        self.pub_ear_mode = rospy.Publisher('/head/ear_cmd', UInt16, queue_size=2)
        self.pub_light_bar = rospy.Publisher('/body/strip_color', UInt32, queue_size=8)

        self.pub_head_pan = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
        self.pub_head_tilt = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)
        self.pub_head_sidetilt = rospy.Publisher('/head_sidetilt_joint/command', Float64, queue_size=1)
        self.pub_right_antenna = rospy.Publisher('/right_antenna_joint/command', Float64, queue_size=1)
        self.pub_left_antenna = rospy.Publisher('/left_antenna_joint/command', Float64, queue_size=1)

        self.pub_wheel_move = rospy.Publisher('/simple_move', SimpleMove, queue_size=1)
        
        rospy.loginfo("%s: init complete." % (self.module_name))

    # ----------------------------------------------------------------


    def imu_callback(self, data): # Degrees
        #print("got imu: ", data) 
        #self.imu_roll = data.x
        #self.imu_pitch = data.y
        self.imu_yaw = data.z

    
    def cleanup(self):
        rospy.loginfo('%s: Cleanup. resetting pose' % self.module_name)
        self.pub_light_bar.publish(0xff000000)        

        # stop the music if it's still going
        if self.play_obj:
            self.play_obj.stop()
        # Turn all the LEDs off
        self.pub_light_bar.publish(0xff000000)        
        
        if not DEBUG_LIGHTS_ONLY:
            # Set Pose back to initial pose
            lowest_servo_speed = 0.3
            goal_pose = self.starting_pose
            self.robotpose.move(goal_pose, lowest_servo_speed)

            # Turn back to starting direction 
            current_angle_rad = normalize_angle_positive(math.radians(self.imu_yaw))
            turn_distance_rad = (shortest_angular_distance(self.start_angle_rad, current_angle_rad)) * -1.0
            turn_distance_deg = math.degrees(turn_distance_rad)
            
            rospy.loginfo("%s: Final turn needed: %f radians, %f degrees" % 
                (self.module_name, turn_distance_rad, turn_distance_deg))
            self.pub_wheel_move.publish(0.0, 0.0, turn_distance_deg, 0.3)            

        self.pub_light_bar.publish(0xff000000)  # Turn lights off      
        self.send_status_update('BODY_LIGHT_MODE', 'STATUS') # Put light bar back into status mode

        rospy.loginfo('%s: Behavior complete' % (self.module_name))

    def slider01_cb(self, value):
        self.slider01 = float(value.data)
        rospy.loginfo('%s: slider01 cb: [%0.2f]' % (self.module_name, self.slider01))        
    def slider02_cb(self, value):
        self.slider02 = float(value.data)
        rospy.loginfo('%s: slider02 cb: [%0.2f]' % (self.module_name, self.slider02))        
        
    def slider03_cb(self, value):
        self.slider03 = float(value.data)
        rospy.loginfo('%s: slider03 cb: [%0.2f]' % (self.module_name, self.slider03))        


    # ----------------------------------------------------------------
    # Converts an audio signal into the int16 format
    def audioToInt(self, audio):
        return (audio * (32767 / np.max(np.abs(audio)))).astype(np.int16)


    # ----------------------------------------------------------------
    def execute_behavior(self, param1, param2):
        rospy.loginfo('%s: Executing behavior' % (self.module_name))
        rospy.loginfo( "Param1: '%s'" % param1) # Song
        rospy.loginfo( "Param2: '%s'" % param2) # Movement Speed TODO - If needed in future

        # DEBUG DEBUG
        dance_offset_adder = 0
        if param2 == 'X' or param2 == '':
            dance_offset_adder = 0
        else:
            try:
                dance_offset_adder = abs(int(param2))
                print("DANCE DEBUG: dance_offset_adder = ", dance_offset_adder)
            except:
                rospy.logwarn('%s: Bad value [%s] for dance_offset_adder. Using 0' % (self.module_name, param2))

        # Make sure AI is off while this behavior runs
        self.pub_ear_mode.publish(self.EAR_CMD_KEYWORD_MODE) # Indicate CMD mode        
        self.ai_enable_pub.publish(False) # Put AI in COMMAND mode        

        # Init new song
        default_lean_amount = 0.25 # 25% lean
        lean_amount = default_lean_amount
        default_lean_lowest_servo_speed = 0.20 # Slower than most servo pose movements
        lean_lowest_servo_speed = default_lean_lowest_servo_speed
        lean_middle_pattern = False
        use_half_bpm = False
        half_bpm_beat_toggle = True # Start on, so the beats skipped are the ODD beats!
        
        song_info = None
        filename = ""
        provided_tempo_bpm = 0
        dance_offset = 0

        #self.speak("Ok")

        #-----------------------------------------------------------------------
        # determine which song to run

        # Test for default param        
        if param1 == 'X' or param1 == '':
            rospy.loginfo('%s: Using default song (Index 0)' % (self.module_name))
            song_info = song_list[DEFAULT_SONG_INDEX]

        # Test for song index param        
        elif param1.isnumeric():
            rospy.loginfo('%s: Selecting song by Index [%s]' % (self.module_name, param1))
            try:
                music_index = abs(int(param1)) - 1 # list is zero offset, but phone starts at 1
                print("DANCE: DBG: music_index = ", music_index)
                if music_index >= 0:
                    song_info = song_list[music_index]
                if song_info is not None:
                    filename = song_info.get("filename")
                    rospy.loginfo('%s: Got song for Index %d [%s]' % (self.module_name, music_index, filename))
            except:
                print("DANCE: DBG: index EXCEPTION")
                pass

        # Test for keyword in param        
        else:
            # Message contains a keyword to search for
            search_phrase = param1.lower()       

            # find song title by keyword search
            # see if there is a match for the entire phrase
            rospy.loginfo("%s: searching for song title with phrase [%s]" % (self.module_name, search_phrase))
            song_info = next((item for item in song_list if search_phrase in item["keywords"]), None)
            if song_info is not None:
                print("found matching Phrase!")
                #print(song_info)
            
            else: # search for longest keyword
                search_word = max(search_phrase.split(), key=len) # use the longest word in the title for search
                print("searching for song title with word ", search_word)
                song_info = next((item for item in song_list if search_word in item["keywords"]), None)
                if song_info is not None:
                    print("found Keyword!")
                    #print(song_info)

        # Final catch in case song not found
        if song_info is None:
            song_info = song_list[DEFAULT_SONG_INDEX]


        #-----------------------------------------------------------------------
        # Got song info, now get the song parameters 
        filename = song_info.get("filename")
        provided_tempo_bpm = song_info.get("bpm")
        dance_offset = song_info.get("dance_offset")
            
        #-----------------------------------------------------------------------
        # Got the song name, now load the file    
        full_filename = filename + '.wav'
        rospy.loginfo('%s: Playing Song [%s]' % (self.module_name, full_filename))
        self.music_path = os.path.join(self.music_dir, full_filename)

        if not os.path.isfile(self.music_path):
            rospy.logwarn("%s: ERROR: %s not found, using default!"% (self.module_name, self.music_path))
            full_filename = DEFAULT_FILE_NAME + '.wav'
            self.music_path = os.path.join(self.music_dir, full_filename)
            #self.speak("I couldnt find that song. Here is another one")
 
        # Load music file       
        # If you get an error, check file permissions of the audio file!
        # Also, some files from Mac don't work. Use audacity to convert to new file
        rospy.loginfo("%s: Loading: %s"% (self.module_name, self.music_path))
            
        self.y, self.sr = librosa.load(self.music_path)
        rospy.loginfo("%s: Audio loaded. "% (self.module_name))

        if not DEBUG_LIGHTS_ONLY:
            self.starting_pose = self.robotpose.get_current_pose() # save pose to restore it later
            rospy.loginfo('%s: Saving Current pose: [%d]' % (self.module_name, self.starting_pose))
            #(self.start_thigh_lift, self.start_knee_bend, self.start_ankle_rotate) = self.get_leg_servo_positions()
            if self.starting_pose < 2:
                # Dance movements requre a standing pose
                pose_lowest_servo_speed = 0.3 
                goal_pose = 3
                self.starting_pose = goal_pose
                self.robotpose.move(goal_pose, pose_lowest_servo_speed)


        # Possible Optimization: Set servo speeds here instead of in the loop
        # (for example: self.robotpose.set_lean_speeds(lowest_servo_speed))

        self.send_status_update('BODY_LIGHT_MODE', 'BEHAVIOR') 

        # Center the head
        servo_speed = 0.35  # 0.2
        SetServoSpeed(servo_speed, neck_joints)
        self.pub_head_pan.publish(0.0)
        self.pub_head_tilt.publish(-0.15) # -0.40 = center for Pose 3 TODO TODO 
        self.pub_head_sidetilt.publish(0.0) 

        # Get initial rotation position
        self.start_angle_rad = normalize_angle_positive(math.radians(self.imu_yaw))
        rospy.loginfo("%s: Start Angle: %f radians, %f degrees" % 
            (self.module_name, self.last_angle_rad, math.degrees(self.last_angle_rad)))

        rospy.loginfo("%s: Detecting Beats..."% (self.module_name))

        # Detect the beats
        beat_times_combi = beat.detect_combi_beats(self.y, self.sr)
        sub_times = beat.detect_beats(self.y, self.sr, freq_range='sub')
        low_times = beat.detect_beats(self.y, self.sr, freq_range='low')
        mid_times = beat.detect_beats(self.y, self.sr, freq_range='mid')
        high_mid_times = beat.detect_beats(self.y, self.sr, freq_range='high_mid')
        high_times = beat.detect_beats(self.y, self.sr, freq_range='high')
        beats = [
            sub_times,
            low_times,
            mid_times,
            high_mid_times,
            high_times
        ]


        # Here we go! Start playing the song!
        rospy.loginfo("%s: Playing Song..."% (self.module_name))
        # Play the audio
        self.play_obj = simpleaudio.play_buffer(self.audioToInt(self.y), 1, 2, self.sr)
        start_time = time.time()
        
            
        # Calculate the *estimated* BPM (not really accurate)
        bpm_starting_est = len(beat_times_combi) / (len(self.y) / self.sr) * 60
        print("Estimated BPM = {0: 6.2f}".format(bpm_starting_est))   
        
        if provided_tempo_bpm > 0.0:
            # Song has tempo provided. Use it instead of estimating.
            bpm_starting_est = provided_tempo_bpm
            print("Song Override BPM = {0: 6.2f}".format(bpm_starting_est))   
             

        if False: # DEBUG_PRINT_BEATS:
            print("        |  S   L   M   H   H   B ")
            print("        |  u   o   i   i   i   e ")
            print("        |  b   w   d   g   g   a ")
            print("        |              h   h   t ")
            print("        |                        ")
            print("        |              M         ")
            print("        |              i         ")
            print("  time  |              d         ")
            print("--------+------------------------")

        # Sync the beats
        beat_indices = [0, 0, 0, 0, 0]
        combi_beat_index = 0

        # seconds / beat = 60 / BPM
        if bpm_starting_est > 120:        
            print("********* FAST SONG ****************")
            print("********* RYTHM SET TO HALF SPEED ****************")
            # Move servos at half beat speed
            use_half_bpm = True
            #print("********* MIDDLE PATTERN ENABLED ****************")
            #lean_middle_pattern = True
                    
        tempo_beat_duration_seconds = 60 / bpm_starting_est 
        first_tempo_beat_found = False
        THRESHOLD_PERCENT = 0.90 # detect tempo beats that occur after the threshold 
        #average_bpm = 0.0
        bpm_calc_first_beat_time  = 0.0
        bpm_calc_samples = 0
        move_direction = 'RIGHT'
        move_beat = False
        half_move_beat_toggle = False
        manual_next_beat_time = start_time
        manual_next_double_beat_time = start_time
        randTurn = 1
        beat_count = (dance_offset+dance_offset_adder) * -1 # start negative to offset
        print("DANCE DEBUG: Starting Beat Count = ", beat_count)
        
        loop_rate = rospy.Rate(20) # 10hz  # EFFECTS ON-TIME OF LEDS     
        while self.play_obj.is_playing() and not rospy.is_shutdown():  

            time_since_start = time.time() - start_time # in seconds
            loop_begin_time = time.time() # in seconds
            move_beat = False
            double_move_beat = False
            
            if DEBUG_PRINT_BEATS:
                print(" {0: 6.2f} | ".format(time_since_start), end=' ')
            #else:
                #print("Time: {0: 6.2f} ".format(time_since_start), end='\r')
                #print("")
                #print("Time: {0: 6.2f} ".format(time_since_start))

            # Turn all the LEDs off until the next beat
            self.pub_light_bar.publish(0xff000000)        


            # Manual beats, if tempo is provided (more accurate)
            if provided_tempo_bpm > 0 and first_tempo_beat_found and time_since_start >= manual_next_beat_time:
                #print("MANUAL BEAT!")
                move_beat = True # tell servos to move
                self.pub_light_bar.publish(0x812f2f2f) # LED 0 and 7 white
                ##self.pub_light_bar.publish(0x813f0000) # LED 0 and 7 red
                manual_next_double_beat_time = manual_next_beat_time + (tempo_beat_duration_seconds / 2.0)
                manual_next_beat_time = manual_next_beat_time + tempo_beat_duration_seconds
                if DEBUG_PRINT_BEATS:
                    print("m", end='   ')
                #else:
                    #print("") # for debugging servo timing
                    #print("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB BEAT BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB")
                    #print("")

            else:
                if DEBUG_PRINT_BEATS:
                    print(" ", end='   ')

            if provided_tempo_bpm > 0 and first_tempo_beat_found and time_since_start >= manual_next_double_beat_time:
                double_move_beat = True # tell head servos to move
                

            for i, beat_times in enumerate(beats):
                if beat_indices[i] < len(beat_times) and time_since_start >= beat_times[beat_indices[i]]:
                    if DEBUG_PRINT_BEATS:
                        print("o", end='   ')
                    beat_indices[i] += 1
                    # got a beat. Handle LEDs from low frequency to high
                    # Example: rostopic pub -1 /body/strip_color std_msgs/UInt32 0xff2f2f2f # 0xff000000
                    # First byte is bit address of the 8 leds, next 3 bytes are R,G,B
                    if True:
                        if i == 0:
                            self.pub_light_bar.publish(0x20301000) # Led 5: Orange: red + green 
                        if i == 1:
                            self.pub_light_bar.publish(0x102f2f00) # Led 4: Yellow: red + green
                        if i == 2:
                            self.pub_light_bar.publish(0x08002f00) # Led 3: Green: 
                        if i == 3:
                            self.pub_light_bar.publish(0x0400002f) # Led 2: Blue
                        if i == 4:
                            self.pub_light_bar.publish(0x022f002f) # Led 1: Purple: red + blue
                else:
                    if DEBUG_PRINT_BEATS:
                        print(" ", end='   ')

            if combi_beat_index < len(beat_times_combi) and time_since_start >= beat_times_combi[combi_beat_index]:
                # print(f"\033[1mCombi {time_since_start}\033[0m")
                if DEBUG_PRINT_BEATS:
                    print("\033[31m\033[1mo\033[0m", end='   ')
                    
                # got the combo beat
                combi_beat_index += 1
                self.pub_light_bar.publish(0x402f0000) # LED 6 red on the combi-beat
                                
                if not first_tempo_beat_found:
                    first_tempo_beat_found = True
                    # Start BPM rhythm, synced to first beat
                    bpm_calc_first_beat_time = time_since_start
                    manual_next_beat_time = time_since_start + tempo_beat_duration_seconds
                    last_tempo_beat_time = time_since_start

                else:
                    # sync tempo to music beat, ignoring odd / syncopated beats
                    time_between_beats = time_since_start - last_tempo_beat_time
                    if time_between_beats > (tempo_beat_duration_seconds * THRESHOLD_PERCENT):
                        # Good Tempo Beat!
                        bpm_calc_samples = bpm_calc_samples + 1.0
                        average_sample_time = (time_since_start - bpm_calc_first_beat_time) / bpm_calc_samples
                        calculated_bpm = 60.0 / average_sample_time
                        
                        last_tempo_beat_time = time_since_start # setup for the next beat

                        if not (provided_tempo_bpm > 0):
                            # Tell servos when to move with automatic beat detector
                            # this is less reliable then provided tempo                       
                            move_beat = True
                            self.pub_light_bar.publish(0x812f2f2f) # LED 0 and 7 white
                            ##self.pub_light_bar.publish(0x813f0000) # LED 0 and 7 red

                        if DEBUG_PRINT_BEATS:
                            print("o", end='   ')
                            print("BPM: {0: 6.2f} | ".format(calculated_bpm), end=' ')
                    else:
                        ## self.pub_light_bar.publish(0x402f0000) # LED 6 red on the off-beat
                        if DEBUG_PRINT_BEATS:
                            print(" ", end='   ')

            else:
                if DEBUG_PRINT_BEATS:
                    print(" ", end='   ')

            #print("next line ", end="\r")
            if DEBUG_PRINT_BEATS:
                print()

            if self.interrupt_check():
                rospy.loginfo('%s: Interrupt Detected. Exiting Behavior.' % (self.module_name))
                break





            # TIME TO DANCE!  Move servos on the beat

            # Head bob 
            if double_move_beat:
                # return head on the half beat (head move faster than body)
                if not DEBUG_LIGHTS_ONLY:
                    self.pub_head_tilt.publish(-0.20) # -0.40 = center
               
            if move_beat:
                if not DEBUG_LIGHTS_ONLY:
                    self.pub_head_tilt.publish(-0.10) 


            if move_beat:
                beat_count = beat_count + 1
                print("BEAT_COUNT = %d <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" 
                    % beat_count)

                if use_half_bpm: # for fast songs
                    if half_bpm_beat_toggle:
                        half_bpm_beat_toggle = False
                        print("use_half_bpm: Skipping a beat")
                        move_beat = False
                        #double_move_beat = False
                    else:
                        half_bpm_beat_toggle = True


 
            # Other Head movement 
            if move_beat:

                if half_move_beat_toggle:
                    half_move_beat_toggle = False
                    if not DEBUG_LIGHTS_ONLY:
                        self.pub_head_sidetilt.publish(0.15) 
                        if beat_count > 3: # first beats are head bob only
                            self.pub_head_pan.publish(-0.2)
                            self.pub_right_antenna.publish(self.ANTENNA_CENTER - 0.4)
                            self.pub_left_antenna.publish(self.ANTENNA_CENTER + 0.4)
                else:
                    half_move_beat_toggle = True
                    if not DEBUG_LIGHTS_ONLY:
                        self.pub_head_sidetilt.publish(-0.15) 
                        if beat_count > 3: # first beats are head bob only
                            self.pub_head_pan.publish(0.2) # pan head half as fast as nod head
                            self.pub_right_antenna.publish(self.ANTENNA_CENTER + 0.4)
                            self.pub_left_antenna.publish(self.ANTENNA_CENTER - 0.4)


            # Robot Body movement  
            if move_beat and beat_count > 7: # Start song with head bob, then add body motion
            
            
                # Time to move to next position
                success = True
                print(">>>>>>>>>>>>> MOVING %s! <<<<<<<<<<<<<<<<" % move_direction)            


                # Sliders for debugging
                # Slider 1 used as switch: pos = normal, neg = middle pattern enabled
                
                pattern_switch = (self.slider01 / 100.0)
                if pattern_switch < 0.0:
                    lean_middle_pattern = True
                    print("LEAN_MIDDLE_PATTERN Enabled")
                else:
                    lean_middle_pattern = False
                                        

                lean_amount = default_lean_amount + (self.slider02 / 100.0)  # lean range from -0.80 to 1.20 % 
                if lean_amount < 0.01: # 1 percent
                    lean_amount = 0.01
                print("LEAN AMOUNT = %0.2f"% lean_amount)

                lean_lowest_servo_speed = default_lean_lowest_servo_speed + (self.slider03 / 100.0)  # speed range  
                if lean_lowest_servo_speed < 0.01:
                    lean_lowest_servo_speed = 0.01
                print("SERVO SPEED = %0.2f"% lean_lowest_servo_speed)
                print()
                # --------------------------

                if not DEBUG_LIGHTS_ONLY:
                    if move_direction == 'LEFT_CENTER' or move_direction == 'RIGHT_CENTER': # using lean_middle_pattern
                        success = self.robotpose.lean2('CENTER_LOW', lean_amount, lean_lowest_servo_speed)

                    else:
                        success = self.robotpose.lean2(move_direction, lean_amount, lean_lowest_servo_speed)

                    if not success:
                        rospy.logwarn('%s: Robotpose.lean error. Exiting Behavior.' % self.module_name)
                        break


                # next pose
                if lean_middle_pattern:
                    if move_direction == 'RIGHT':                
                        move_direction = 'LEFT_CENTER'
                    elif move_direction == 'LEFT_CENTER':                
                        move_direction = 'LEFT'
                    elif move_direction == 'LEFT':                
                        move_direction = 'RIGHT_CENTER'
                    elif move_direction == 'RIGHT_CENTER':                
                        move_direction = 'RIGHT'

                else:
                    if move_direction == 'RIGHT':
                        move_direction = 'LEFT'
                    elif move_direction == 'LEFT':
                        move_direction = 'RIGHT'

                print("Move direction set to: %s" % move_direction)


            #if move_beat and beat_count > (13*8+1): # DEBUG REMOVE THIS
            #    print("                             XXXXXXXXXXXXXXXXXXXXXXXXXx")
            #    break





            # Robot Wheels movement
            # beat counts may be even or odd, due to dropping a beat for high BPM
            spin1_start_beat = int(10*8)
            
            turn_beats = 36 # worst case, fast song 
            if provided_tempo_bpm > 0:
                turn_beats =  int(provided_tempo_bpm * 13 / 60) # 13 seconds to turn

            spin1_end_beat = spin1_start_beat + turn_beats

            set2_start = int(spin1_end_beat)
            spin2_end_beat = ((9*8) + set2_start) + turn_beats
            set3_start = int(spin2_end_beat + 1)

            print("DBG: TURN BEATS = %d, spin1 end beat = %d set2_start = %d MOVE2 = %d" % 
                (turn_beats, spin1_end_beat, set2_start, ((1*8) + set2_start) ) )

              
            if move_beat: 

                move1 = ((1*8) + set2_start)
                move2 = ((2*8) + set2_start)
                move3 = ((3*8) + set2_start)
                move4 = ((4*8) + set2_start)

                move5 = ((1*8) + set3_start)
                move6 = ((2*8) + set3_start)
                move7 = ((3*8) + set3_start)
                move8 = ((4*8) + set3_start)




                print("")            
                print("START2 DEBUG =============================================================")
                print("beat_count = %d, beat_count+1 = %d" % (beat_count, beat_count+1))
                print("move1 = ", move1)
                print("move2 = ", move2)
                print("move3 = ", move3)
                print("move4 = ", move4)

                print("move3-1 = ", move5)
                print("move3-2 = ", move6)
                print("move3-3 = ", move7)
                print("move3-4 = ", move8)
                print("START2 DEBUG =============================================================")
                print("")            


 
                if  (beat_count  == (1*8) or beat_count   ==  (1*8) + set2_start or beat_count   ==  (1*8) + set3_start or
                    beat_count+1 == (1*8) or beat_count+1 ==  (1*8) + set2_start or beat_count+1 ==  (1*8) + set3_start):
                    self.pub_wheel_move.publish(0.1, 0.3, 0.0, 0.0)

                elif (beat_count == (2*8) or beat_count   ==  (2*8) + set2_start or beat_count   ==  (2*8) + set3_start or
                    beat_count+1 == (2*8) or beat_count+1 ==  (2*8) + set2_start or beat_count+1 ==  (2*8) + set3_start):
                    print("DANCE DEBUG: MOVE WHEELS BACK") # Back to starting position
                    self.pub_wheel_move.publish(-0.1, 0.3, 0.0, 0.0)


                elif (beat_count == (3*8) or beat_count   ==  (3*8) + set2_start or beat_count   ==  (3*8) + set3_start or
                    beat_count+1 == (3*8) or beat_count+1 ==  (3*8) + set2_start or beat_count+1 ==  (3*8) + set3_start):
                    print("DANCE DEBUG: MOVE WHEELS TURN") 
                    randTurn = random.choice((-1, 1)) # Pick turn direction (so all turns aren't the same)
                    self.pub_wheel_move.publish(0.0, 0.0, (60.0*randTurn), 0.3)

                elif (beat_count == (4*8) or beat_count   ==  (4*8) + set2_start or beat_count   ==  (4*8) + set3_start or
                    beat_count+1 == (4*8) or beat_count+1 ==  (4*8) + set2_start or beat_count+1 ==  (4*8) + set3_start):
                    print("DANCE DEBUG: MOVE WHEELS TURN BACK") 
                    self.pub_wheel_move.publish(0.0, 0.0, (-60.0*randTurn), 0.3)


                elif (beat_count == (5*8) or beat_count   ==  (5*8) + set2_start or beat_count   ==  (5*8) + set3_start or
                    beat_count+1 == (5*8) or beat_count+1 ==  (5*8) + set2_start or beat_count+1 ==  (5*8) + set3_start):
                    print("DANCE DEBUG: MOVE WHEELS OPPOSITE TURN") 
                    self.pub_wheel_move.publish(0.0, 0.0, (-60.0*randTurn), 0.3)

                elif (beat_count == (6*8) or beat_count   ==  (6*8) + set2_start or beat_count   ==  (6*8) + set3_start or
                    beat_count+1 == (6*8) or beat_count+1 ==  (6*8) + set2_start or beat_count+1 ==  (6*8) + set3_start):
                    print("DANCE DEBUG: MOVE WHEELS TURN BACK")  # Back to starting position
                    self.pub_wheel_move.publish(0.0, 0.0, (60.0*randTurn), 0.3)


                elif (beat_count == (7*8) or beat_count   ==  (7*8) + set2_start or beat_count   ==  (7*8) + set3_start or
                    beat_count+1 == (7*8) or beat_count+1 ==  (7*8) + set2_start or beat_count+1 ==  (7*8) + set3_start):
                    print("DANCE DEBUG: MOVE WHEELS BACK") 
                    self.pub_wheel_move.publish(-0.1, 0.3, 0.0, 0.0)

                elif (beat_count == (8*8) or beat_count   ==  (8*8) + set2_start or beat_count   ==  (8*8) + set3_start or
                    beat_count+1 == (8*8) or beat_count+1 ==  (8*8) + set2_start or beat_count+1 ==  (8*8) + set3_start):
                    print("DANCE DEBUG: MOVE WHEELS FORWARD")  # Back to starting position
                    self.pub_wheel_move.publish(0.1, 0.3, 0.0, 0.0)


                elif (beat_count == (9*8) or beat_count   ==  (9*8) + set2_start or beat_count   ==  (9*8) + set3_start or
                    beat_count+1 == (9*8) or beat_count+1 ==  (9*8) + set2_start or beat_count+1 ==  (9*8) + set3_start):
                    print("DBG: SPIN START SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSs")
                    randSpin = random.choice((-1, 1)) # every spin is random direction
                    self.pub_wheel_move.publish(0.0, 0.0, (359.0*randSpin), 0.3) # Full circle!



            if self.interrupt_check():
                rospy.loginfo('%s: Interrupt Requested. Exiting Behavior.' % (self.module_name))
                break

            loop_rate.sleep() # use rospy's loop rate function
            # ---------------- end of loop -------------------------------------



        # End behavior
        rospy.loginfo('%s: Behavior ending...' % (self.module_name))
        if not DEBUG_LIGHTS_ONLY:
            final_direction = 'CENTER_HIGH'
            success = self.robotpose.lean2(final_direction, lean_amount, lean_lowest_servo_speed)
            if not success:
                rospy.logwarn('%s: Robotpose.lean error.' % self.module_name)
 
        self.cleanup()         

       
if __name__ == '__main__':
    rospy.init_node('dance_behavior')
    server = DanceBehavior()
    rospy.spin()
