#! /usr/bin/env python3
# Speech Recognition Publisher
# Uses streaming for faster Text to speech response (process sound in chunks as receieved)

# This version runs in "One Shot mode", meaning:
#   Wait for Listen command
#   Translate until pause in talking
#   Return results
#   Stop listening until next Listen command

# Utilizes streaming example from Google:  
# https://cloud.google.com/speech-to-text/docs/transcribe-streaming-audio#speech-streaming-recognize-python

import sys
import os
import signal
import time
import pyaudio
from google.cloud import speech as speech

# ROS
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from system_status_msgs.msg import SystemStatus

# Local includes
from microphone import find_microphone
from keyword_detector import KeywordDetector
from microphone_stream import MicrophoneStream


# ====================================
# For Porcupine Keyword spotting
# import argparse

# Set Env variable for google speech
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "/home/system/key.json"


# CONSTANTS
DEFAULT_TIMEOUT = 15.0 # seconds
ONE_SHOT_MODE = True
LANGUAGE_SUPPORTS=["en-US"]


# Audio recording parameters
SAMPLE_RATE = 16000 # was 44100
CHUNK = 512 # int(SAMPLE_RATE / 10)  # 100ms?

# GLOBALS
g_exit_request = False

def signal_handler(signal, frame):
    global g_exit_request
    g_exit_request = True

def interrupt_check():
    #global g_exit_request
    #print("INTERRUPT_CHECK Called")
    if g_exit_request:
        #print("RETURNING INTERRUPT TRUE")
        return True
    else:
        #print("RETURNING INTERRUPT FALSE")
        return False

# ======================================================================================


class SystemStatusUpdate():
    def __init__(self):
        self.status_pub = rospy.Publisher('/system_status', SystemStatus, queue_size=8)

    def send_status_update(self, item, status):
        # Send status update to system monitor
        status_msg = SystemStatus()
        status_msg.item = item
        status_msg.status = status
        self.status_pub.publish(status_msg)

# =================================================================================================================
class StreamingSpeechReco(object):

    def __init__(self):
       
        self.logname = 'SpeechReco: '
        rospy.init_node('streaming_speech_recognition', anonymous=True)
        self.interrupt_check = interrupt_check
        self.stream = None
        self.send_status_update = SystemStatusUpdate().send_status_update
        self.listen = False
        self.microphone_index = -1
        self.keyword_mode = False
        self.streaming_recogniton_enabled = True
        self.timeout = rospy.get_param('listen_timeout', DEFAULT_TIMEOUT)
        self.generator_start_time = None

        language_supports = LANGUAGE_SUPPORTS
        self.language_code = language_supports[0]  # a BCP-47 language tag
        # alternative_language_codes = []
        # if len(self._language_supports) > 1:
        #     alternative_language_codes = self._language_supports[1:]



        # Get PicoVoice access key from the os environment:
        try:
            self.pico_voice_access_key = os.environ['PICO_VOICE_ACCESS_KEY']
            print("PICO_VOICE_ACCESS_KEY = [%s]" % self.pico_voice_access_key)
        except KeyError:
            rospy.logwarn("Keyword Detector FAIL: [PICO_VOICE_ACCESS_KEY] probably not set in .bashrc")
        
        
        # SUBSCRIBERS
        rospy.Subscriber("/speech_recognition/mode", String, self.mode_callback)
        rospy.Subscriber("/speech_recognition/listen", Bool, self.listen_callback)

        
        # PUBLISHERS       
        self.speech_reco_pub = rospy.Publisher('/speech_recognition/results', String, queue_size=8)
    

        # Make sure we can access the desired microphone
        # NOTE: If Ubuntu Setting are open on Sound page, it can block mic access! 
        self.microphone_index = find_microphone()
        if self.microphone_index >= 0:
            self.send_status_update("SPEECH_RECO_STATE", "MIC_READY")
            rospy.loginfo("%s: Find Microphone SUCCESS: Index = %d"  % (self.logname, self.microphone_index))
        else:
            rospy.loginfo("%s: Find Microphone FAILED! Index = %d"  % (self.logname, self.microphone_index))
            self.send_status_update("SPEECH_RECO_STATE", "MIC_FAIL")

        rospy.loginfo("%s: Init complete." % (self.logname))


    # Callback function so other modules can query state
    def is_keyword_mode(self):
        return self.keyword_mode

    def listen_callback(self, data):
        # When AI wants a response, it sends this listen message
        # Otherwise, we don't listen except on keyword mode
        listen = data.data
        if listen:
            if self.streaming_recogniton_enabled:
                print()
                print()
                print("*****************************************************************************")
                rospy.loginfo("%s: Listen request received. Streaming Reco is Listening" % (self.logname))
                self.listen = True
            else:
                rospy.loginfo("%s: Listen request IGNORED. Streaming Recogniton DISABLED." % (self.logname))
                self.listen = False
        else:
            rospy.loginfo("%s: Listen request False received. Streaming Reco is not Listening" % (self.logname))
            self.listen = False


    def mode_callback(self, data):
        mode = data.data.lower()
        rospy.loginfo("%s: New mode requested: [%s]" % (self.logname, mode))
        if mode == 'disable':
            self.streaming_recogniton_enabled = False
            self.keyword_mode = False
                    
        elif mode == 'keyword':
            self.streaming_recogniton_enabled = False
            self.keyword_mode = True
                   
        elif mode == 'stream':
            self.streaming_recogniton_enabled = True
            self.keyword_mode = False
 
        else:
            rospy.logwarn('%s: BAD mode requested: [%s], Ignored.' % (self.logname, mode))
        

    def time_expired(self):
        if self.generator_start_time == None:
            return False
            
        elapsed_time = time.time() - self.generator_start_time # in seconds
        # print("DEBUG: Elapsed_time = %2.2f" % elapsed_time)

        if elapsed_time > self.timeout:
            print("==============================================================")
            rospy.loginfo("SpeechReco: Listen Time expired")
            print("==============================================================")
            return True

        else:
            return False



    def listenPrintLoop(self, responses, stream):
        """Iterates through server responses and prints them.

        The responses passed is a generator that will block until a response
        is provided by the server.

        Each response may contain multiple results, and each result may contain
        multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
        print only the transcription for the top alternative of the top result.

        In this case, responses are provided for interim results as well. If the
        response is an interim one, print a line feed at the end of it, to allow
        the next result to overwrite it, until the response is a final one. For the
        final one, print a newline to preserve the finalized transcription.
        """

        num_chars_printed = 0
        finaltext = ""
        for response in responses:

            if self.keyword_mode:
                return  # Exit the loop

            if not response.results:
                continue

            # The `results` list is consecutive. For streaming, we only care about
            # the first result being considered, since once it's `is_final`, it
            # moves on to considering the next utterance.
            result = response.results[0]
            if not result.alternatives:
                continue

            # Display the transcription of the top alternative.
            transcript = result.alternatives[0].transcript

            # Display interim results, but with a carriage return at the end of the
            # line, so subsequent lines will overwrite them.
            #
            # If the previous result was longer than this one, we need to print
            # some extra spaces to overwrite the previous result
            overwrite_chars = ' ' * (num_chars_printed - len(transcript))

            # Display text in system monitor as it comes in (not final, then final)
            self.send_status_update("SPEECH_RECO_TEXT", transcript)

            if not result.is_final:
                # DBG:
                sys.stdout.write('SpeechReco:Partial: ' + transcript + overwrite_chars + '\r')
                sys.stdout.flush()

                num_chars_printed = len(transcript)

            else:
                finaltext = transcript + overwrite_chars
                finaltext = finaltext.lstrip() # remove leading spaces

                print("")
                print("******************************************************")
                print("SpeechReco:  Final: ", finaltext)
                print("******************************************************")
                print("")
                
                ##if self.keyword_mode:
                ## TODO Remove? in keyword mode, google stream still gets audio to keep it open, but we ignore what it hears
                ##    finaltext = ''
                
                if finaltext == "" and not self.time_expired():
                    print("DBG:SpeechReco: Got blank finaltext before timer expired. Retrying...")
                else:
                    break

        # Send response. Empty string indicates timeout occured                    
        self.speech_reco_pub.publish(finaltext)
        print("DEBUG: Publishing: SpeechReco:  Final: ", finaltext)
        self.listen = False
        self.send_status_update("SPEECH_RECO_STATE", "LISTEN_DONE")

        #print("===========================================================")
        #print("DEBUG: Exit ListenPrintLoop!" )
        #print("===========================================================")
                


    def run(self):

        if self.microphone_index < 0:
            for i in range(0, 5):
                rospy.logfatal("%s:run: NO MICROPHONE FOUND! SPEECH RECOGNITION EXITING!" % (self.logname))
                self.send_status_update("SPEECH_RECO_STATE", "MIC_FAIL")
                rospy.sleep(0.5) 
            return
            
        # Setup for Streaming recognition loop
        rospy.loginfo("%s: DBG: run: Configuring SpeechClient" % (self.logname))

        client = speech.SpeechClient()
        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=SAMPLE_RATE,
            language_code=self.language_code,
            enable_automatic_punctuation=True
            # alternative_language_codes=alternative_language_codes
        )
        streaming_config = speech.StreamingRecognitionConfig(
            config=config,
            interim_results=True)
            

        rospy.loginfo("%s: DBG: run: About to start loop" % (self.logname))
        status_update_counter = 0
        while not rospy.is_shutdown() and not self.interrupt_check():

            if not self.keyword_mode and not (self.streaming_recogniton_enabled and self.listen):
                # Standing by. Update status occasionally
                status_update_counter = status_update_counter + 1
                if status_update_counter > 10:
                    status_update_counter = 0
                    if self.streaming_recogniton_enabled:
                        self.send_status_update("SPEECH_RECO_STATE", "READY")
                    else:
                        self.send_status_update("SPEECH_RECO_STATE", "DISABLED")
                
                rospy.sleep(0.1) # let ROS threads run while we wait
                continue

            rospy.loginfo("%s: DBG: run: top of loop" % (self.logname))


            rospy.loginfo("%s: DBG: run: Creating Microphone Stream" % (self.logname))
            with MicrophoneStream(SAMPLE_RATE, CHUNK, self.microphone_index, self.time_expired,
                interrupt_check, self.is_keyword_mode) as stream:
                self.stream = stream

                if self.keyword_mode:
                    # Keyword Mode
                    rospy.loginfo("%s: DBG: ************************************************************************" % (self.logname))
                    rospy.loginfo("%s: DBG: run: Keyword Mode. Using keyword loop, not Google Streaming Recognition!" % (self.logname))
                    self.send_status_update("SPEECH_RECO_STATE", "KEYWORD_RUNNING")
                    stream.keyword_monitor() # runs as long as we are in keyword mode
                    rospy.loginfo("%s: DBG: run: ************ : Keyword Loop EXIT! **************" % (self.logname))

                else: # if self.streaming_recogniton_enabled:
                    # Streaming Mode

                    rospy.loginfo("%s: DBG: run: Starting stream.generator()" % (self.logname))
                    audio_generator = stream.generator()

                    rospy.loginfo("%s: DBG: run: creating requests with generator" % (self.logname))
                    requests = (speech.StreamingRecognizeRequest(audio_content=content)
                                for content in audio_generator)
                    self.send_status_update("SPEECH_RECO_STATE", "LISTENING")
                    self.generator_start_time = time.time()
                    rospy.loginfo("%s: DBG: run: starting requests with generator" % (self.logname))
                    responses = client.streaming_recognize(streaming_config, requests)

                    # print('******************* GOOGLE SPEECH TO TEXT RESPONSES ******************')
                    rospy.loginfo("%s: Begin run try block." % (self.logname))
                    try:
                        rospy.loginfo("%s: DBG: run: calling ListenPrintLoop" % (self.logname))
                        # pass the hearing callback function to the ListenPrintLoop
                        self.listenPrintLoop(responses, stream)
                        rospy.loginfo("%s: DBG: run: ListenPrintLoop Exit" % (self.logname))

                    except Exception as ex:
                        # Most likely stream too long exception, so just log and restart
                        print("**************************************************************")
                        rospy.loginfo("%s:run: Exception from Google Streaming Speech to Text" % (self.logname))
                        print(ex)
                        #pass
                    #  except KeyboardInterrupt:
                    #    print('You have pressed ctrl-c button.')
                    

        rospy.loginfo("%s: AI Node:run: Shutting Down." % (self.logname))
        
        
        
 # ------------------------------------------------------------------
if __name__ == '__main__':

    # capture SIGINT signal, e.g., Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
 
    try:
        reco_engine = StreamingSpeechReco()
        reco_engine.run()

    except rospy.ROSInterruptException:
        pass
       
        
        
        
        
        
        
        
        
        
        
        
