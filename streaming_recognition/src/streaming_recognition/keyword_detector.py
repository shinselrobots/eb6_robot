#! /usr/bin/env python3
# Speech Recognition Publisher using:
#     Porcupine keyword spotter (robot's name, microphone on)
#     Python SpeechRecognition framework
#     Google Cloud Speech API for command / phrase recognition
# Listens to the microphone for a Keyword, then sends text to google cloud for decoding
##
# Information on installing the speech recognition library can be found at:
# https://pypi.python.org/pypi/SpeechRecognition/

# Publishes messages:
#   speech_recognition/text
#   Eye color, Ear mode command


import rospy
import actionlib
##import robot_sounds.msg
import time
import sys
import signal
import speech_recognition as kwsr # keyword speech recognition
import os
import urllib.request # internet detection

import rospkg
import rosparam

##from speech_recognition_common.srv import *  # speech_recognition_intent
from std_msgs.msg import Bool
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Float64
from behavior_msgs.msg import CommandState 
from system_status_msgs.msg import SystemStatus


# ====================================
# For Porcupine
import argparse
import os
import struct
import wave
from datetime import datetime
from threading import Thread

import pvporcupine
# from pvrecorder import PvRecorder



# ====================================
# globals
interrupted = False
SPEECH_RECO_NOT_LISTENING = 0
SPEECH_RECO_LISTENING = 1
SPEECH_RECO_IDLE = 2

# Change these to match your system configuration
DEFAULT_RESOURCE_DIR = '/home/system/catkin_robot/resources/'
DEFAULT_KEYPHRASE_1 = 'Eee-Bee-Six_en_linux_v2_1_0.ppn'   # Robot's name
DEFAULT_KEYPHRASE_2 = 'microphone-on_en_linux_v2_1_0.ppn' # Microphone On


def signal_handler(signal, frame):
    global interrupted
    interrupted = True

def interrupt_callback():
    #print("got interrupt_callback")
    return interrupted

## TODO IS THIS USED?
class ListenStateUpdate():
    def __init__(self):
        self.listen_state_pub = rospy.Publisher('/listen_state', UInt16, queue_size=8)

    def send_listen_state(self, state):
        # Send listen state to other modules (like Antenna movement)
        self.listen_state_pub.publish(state)

class SystemStatusUpdate():
    def __init__(self):
        self.status_pub = rospy.Publisher('/system_status', SystemStatus, queue_size=8)

    def send_status_update(self, item, status):
        # Send status update to system monitor
        status_msg = SystemStatus()
        status_msg.item = item
        status_msg.status = status
        self.status_pub.publish(status_msg)


class KeywordDetector(object):

    def __init__(self):

        self.logname = 'AI Keyword Detector: '
        self.porcupine = None
        self.wav_file = None
        self.saving_wav_audio = False  # keep track of state
        self.audio_samples_count = 0
        self.output_path = ''
        self.keywords = None
        self.wav_file_ready = False
        self.send_listen_state = ListenStateUpdate().send_listen_state
        self.send_status_update = SystemStatusUpdate().send_status_update

        # PUBLISHERS
        self.behavior_cmd_pub = rospy.Publisher('behavior/cmd', CommandState, queue_size=10)

        # PICOVOICE KEYWORD DETECTOR
        # Get PicoVoice access key from the os environment. Usually, this is set in .bashrc
        self.pico_voice_access_key = ''
        try:
            self.pico_voice_access_key = os.environ['PICO_VOICE_ACCESS_KEY']
            print("PICO_VOICE_ACCESS_KEY = [%s]" % self.pico_voice_access_key)
        except KeyError:
            rospy.logwarn("Keyword Detector FAIL: [PICO_VOICE_ACCESS_KEY] not set!")

        # Location of keyphrase files, used by picovoice
        self.resource_dir = rospy.get_param('resource_dir', DEFAULT_RESOURCE_DIR)
        keyphrase_1 = rospy.get_param('keyphrase_1', DEFAULT_KEYPHRASE_1)
        keyphrase_2 = rospy.get_param('keyphrase_2', DEFAULT_KEYPHRASE_2)

        # build the full path to the files
        keyphrase_dir = os.path.join(self.resource_dir, 'speech_recognition/')
        keyphrase_1_path = keyphrase_dir +  keyphrase_1
        keyphrase_2_path = keyphrase_dir +  keyphrase_2
        print("Keyword Detector: Keyphrase 1 = ", keyphrase_1_path) 
        print("Keyword Detector: Keyphrase 2 = ", keyphrase_2_path) 

        # hotword_sensitivity = 0.80 # Larger = more sensitive (0.38?)
        apply_frontend = True  # NOT USED Keyword front end noise processing?
        library_path = pvporcupine.LIBRARY_PATH
        model_path = pvporcupine.MODEL_PATH
        sensitivities = None

        keyword_paths = [keyphrase_1_path, keyphrase_2_path]
        if sensitivities is None:
            sensitivities = [0.5] * len(keyword_paths)

        if len(keyword_paths) != len(sensitivities):
            raise ValueError('Number of keywords does not match the number of sensitivities.')

        self.keywords = list()
        for x in keyword_paths:
            keyword_phrase_part = os.path.basename(x).replace('.ppn', '').split('_')
            if len(keyword_phrase_part) > 6:
                self.keywords.append(' '.join(keyword_phrase_part[0:-6]))
            else:
                self.keywords.append(keyword_phrase_part[0])

        try:
            self.porcupine = pvporcupine.create(
                access_key=self.pico_voice_access_key,
                library_path=library_path,
                model_path=model_path,
                keyword_paths=keyword_paths,
                sensitivities=sensitivities)

        except pvporcupine.PorcupineInvalidArgumentError as e:
            print("One or more arguments provided to Porcupine is invalid: ", args)
            print("If all other arguments seem valid, ensure that '%s' is a valid AccessKey" % args.access_key)
            raise e
        except pvporcupine.PorcupineActivationError as e:
            print("AccessKey activation error")
            raise e
        except pvporcupine.PorcupineActivationLimitError as e:
            print("AccessKey '%s' has reached it's temporary device limit" % args.access_key)
            raise e
        except pvporcupine.PorcupineActivationRefusedError as e:
            print("AccessKey '%s' refused" % args.access_key)
            raise e
        except pvporcupine.PorcupineActivationThrottledError as e:
            print("AccessKey '%s' has been throttled" % args.access_key)
            raise e
        except pvporcupine.PorcupineError as e:
            print("Failed to initialize Porcupine")
            raise e

        print("DBG:  >>>> KeyWordDetector initialized.")

    def process_chunk(self, pcm):

        # print("DBG: >>>>  KeyWordDetector: process_chunk")

        # result is index of which keyword was spotted
        result = -1

        pcm = struct.unpack_from("h" * self.porcupine.frame_length, pcm)

        # RUN THE KEYWORD DETECTOR
        result = self.porcupine.process(pcm)
        #print("DBG: >>>>  KeyWordDetector: porcupine done")

        # Index < 0 indicates no keyword spotted
        # Index 0 indicates robot name
        # Index > 0 is immediate command (E.G. "turn microphone on"), so no audio callback needed.

        if result >= 0:
            print('[%s] Detected %s' % (str(datetime.now()), self.keywords[result]))
            print("DBG: >>>>>>>>>>>>>>>>>>>>>>>  KEYWORD DETECTED!!! <<<<<<<<<<<<<<<<<<<<<")
            print("     >>>>>>>>>>>>>>>>>>>>>>>  KEYWORD DETECTED!!! <<<<<<<<<<<<<<<<<<<<<")
            print("     >>>>>>>>>>>>>>>>>>>>>>>  KEYWORD DETECTED!!! <<<<<<<<<<<<<<<<<<<<<")
            print("")
            #TODO DAVE self._ears.set_mode(EAR_CMD_KEYWORD_MODE)
            # Move antennas to immediately indicate listening for more
            #self._ears.move_antenna(self._ears.ANTENNA_LISTENING)
            
            # Light body light to indicate Keyword heard, listening for more!
            # Note, this works even when robot is in sleep mode
            self.send_status_update("SPEECH_RECO_STATE", "KEYWORD_DETECTED")
            
            #TODO move this to sysmon!  Remove from Face Tracker Too!!!
            ## self.send_listen_state("SPEECH_RECO_LISTENING")


        if result == 0:  # robot name called. Save audio.
            print('[%s] Detected Robot Name: %d %s' % (str(datetime.now()), result, self.keywords[result]))

            # save audio into a file to pass to cloud speech recognition
            self.saving_wav_audio = True  # start recording

            # if already saving, throw away prior sample and send new one instead
            if self.wav_file is not None:
                self.wav_file.close()

            self.output_path = '/tmp/porcupine_' + str(int(time.time())) + '.wav'
            self.wav_file = wave.open(self.output_path, "w")
            # number of channels, sample width, framerate, nframes, comptype, compname
            self.wav_file.setparams((1, 2, 16000, 512, "NONE", "NONE"))

            print('[%s] Saving Audio to %s' % (str(datetime.now()), self.output_path))

            # number of chunks to record after the keyword
            # each chunk is 0.032 seconds
            self.audio_samples_count = 50

        if self.saving_wav_audio:
            if self.audio_samples_count > 0:
                self.wav_file.writeframes(struct.pack("h" * len(pcm), *pcm))
                print('[%s] Saving Audio %d' % (str(datetime.now()), self.audio_samples_count))
                self.audio_samples_count = self.audio_samples_count - 1
            else:
                # Done with all samples. Close the audio file and send to speech recognition callback
                self.saving_wav_audio = False  # done
                self.wav_file.close()
                self.wav_file = None
                print('[%s] Done saving audio to %s. Calling callback.' % (str(datetime.now()), self.output_path))
                # audio_callback(self.output_path)
                # self.output_path = ''
                # IMPORTANT TODO: the consumer must delete the temporary audio recording
                # os.remove(fname)
                self.wav_file_ready = True
                # Move antennas to immediately indicate recording done
                #self._ears.move_antenna(self._ears.ANTENNA_NOT_LISTENING)
                self.send_listen_state(SPEECH_RECO_NOT_LISTENING)

        if self.wav_file_ready:
            # This will turn the keyword detected LED back off in sleep mode
            self.send_status_update("SPEECH_RECO_STATE", "KEYWORD_RUNNING")

            self.handle_keyword_audio(self.output_path)
            # Note that this blocks the thread until done.
            self.wav_file_ready = False

        return result

    def handle_keyword_audio(self, fname):

        # Convert audio to text, then see if we recognize the command from the user
        phrase_heard = ''
        print("Keyword Detector: converting audio to text...")
        r = kwsr.Recognizer()
        with kwsr.AudioFile(fname) as source:
            audio = r.record(source)  # read the entire audio file

         # =====================================================
         # Recognize speech using Google Speech Recognition
        try:
            # for testing purposes, we're just using the default API key
            # to use another API key, use `r.recognize_google(audio,
            #     key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
            # instead of `r.recognize_google(audio)`
            phrase_heard = r.recognize_google(audio)
            rospy.loginfo(self.logname + "phrase_heard:  [" + phrase_heard + "]")

        except kwsr.UnknownValueError:
            rospy.loginfo(self.logname + "Handle Keyword: Cloud could not understand audio")
          
        except kwsr.RequestError as e:
            rospy.logwarn(self.logname +
                "Handle Keyword: Could not request results from Google Speech Recognition service")

        # Delete the temporary audio recording
        os.remove(fname)


        # Now, see if command found, and send to behavior server. If no command 
        # recognized, send "NAME_ONLY" which which can be used to "unpause" the AI

        command = ''
        param1 = ''
        param2 = ''

        phrase_heard_uppercase = phrase_heard.upper()
        if phrase_heard_uppercase.find("WAKE") != -1:
            command = "WAKE"
        else:
            # Ignore anything else said, and just acknowledge that we heard our name
            command = "NAME_ONLY"
          
        print("Keyword Detector: Sending behavior command: %s parm1: %s parm2: %s" % (command, param1, param2))
        msg = CommandState()
        msg.commandState = command
        msg.param1 = param1
        msg.param2 = param2
        self.behavior_cmd_pub.publish(msg)

        print("Keyword Detector: Done handling Keyword audio." )
  
 

    def cleanup(self):
        if self.porcupine is not None:
            self.porcupine.delete()

        if self.wav_file is not None:
            self.wav_file.close()
