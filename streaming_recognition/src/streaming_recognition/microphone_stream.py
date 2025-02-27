#! /usr/bin/env python3
# Speech Recognition Publisher
# Utilizes streaming example from Google:  
# https://cloud.google.com/speech-to-text/docs/transcribe-streaming-audio#speech-streaming-recognize-python

import re
import sys
import os
import signal
import time
# from threading import Thread

from google.cloud import speech as speech
import pyaudio
from six.moves import queue

# ROS
import rospy
from std_msgs.msg import String
from system_status_msgs.msg import SystemStatus

# Local includes
from microphone import find_microphone
from keyword_detector import KeywordDetector


os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "/home/system/key.json"

# CONSTANTS Audio recording parameters
SAMPLE_RATE = 16000 # was 44100
CHUNK = 512 # int(SAMPLE_RATE / 10)  # 100ms?


class SystemStatusUpdate():
    def __init__(self):
        self.status_pub = rospy.Publisher('/system_status', SystemStatus, queue_size=8)

    def send_status_update(self, item, status):
        # Send status update to system monitor
        status_msg = SystemStatus()
        status_msg.item = item
        status_msg.status = status
        self.status_pub.publish(status_msg)


class MicrophoneStream(object):

    # Opens a recording stream as a generator yielding the audio chunks.
    def __init__(self, rate, chunk, microphone_index, time_expired, interrupt_check, is_keyword_mode ):
        rospy.loginfo("***************** MicrophoneStream INIT CALLED! *****************")
        self.chunk = chunk
        self.microphone_index = microphone_index
        
        # callback functions
        self.time_expired = time_expired
        self.interrupt_check = interrupt_check
        self.is_keyword_mode = is_keyword_mode

        # Create a thread-safe buffer of audio data
        self.buff = queue.Queue()
        self.closed = True
        self.keyword_detector = None
        self.send_status_update = SystemStatusUpdate().send_status_update
        self.keyword_detector = KeywordDetector()
        

    def __enter__(self):

        if self.microphone_index < 0:
            rospy.loginfo("SpeechReco:MicrophoneStream: NO MICROPHONE FOUND! SPEECH RECOGNITION EXITING!")
            self.send_status_update("SPEECH_RECO_STATE", "MIC_FAIL")
            return

        self.audio_interface = pyaudio.PyAudio()

        rospy.loginfo("SpeechReco:MicrophoneStream: Found preferred microphone device! Starting Speech recognition")
        self.send_status_update("SPEECH_RECO_STATE", "MIC_READY")
        self.audio_stream = self.audio_interface.open(
            format=pyaudio.paInt16,
            channels = 1,
            rate = SAMPLE_RATE,
            input_device_index = self.microphone_index,
            input=True, 
            frames_per_buffer=self.chunk,
            # Run the audio stream asynchronously to fill the buffer object.
            # This is necessary so that the input device's buffer doesn't
            # overflow while the calling thread makes network requests, etc.
            stream_callback=self.fill_buffer,
        )

        self.closed = False
        return self

    def __exit__(self, type, value, traceback):
        rospy.loginfo("AI Node:MicrophoneStream: __exit__ called.")
        self.audio_stream.stop_stream()
        self.audio_stream.close()
        self.closed = True
        # Signal the generator to terminate so that the client's
        # streaming_recognize method will not block the process termination.
        self.buff.put(None)
        self.audio_interface.terminate()

    def fill_buffer(self, in_data, frame_count, time_info, status_flags):
        # Continuously collect data from the audio stream, into the buffer.
        self.buff.put(in_data)
        return None, pyaudio.paContinue


    def keyword_monitor(self):
        # Used in Keyword Only mode
        while self.is_keyword_mode() and not self.closed and not self.interrupt_check():
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self.buff.get()
            if chunk is None:
                print("")
                rospy.loginfo("SpeechReco:MicrophoneStream::Keyword Monitor: chunk exit ")
                return

            # rospy.loginfo("DBG: Keyword Detector is listening ")
            keyword_result = self.keyword_detector.process_chunk(chunk)  # send chunk to keyword detector
            if keyword_result == 0:
                rospy.loginfo("SpeechReco:MicrophoneStream: Robot Name Keyword Detected <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")

            # Now consume whatever other data's still buffered.
            while self.is_keyword_mode() and not self.closed:
                try:
                    chunk = self.buff.get(block=False)
                    if chunk is None:
                        return

                    # Continue getting samples in keyword_detector to get full command
                    keyword_result = self.keyword_detector.process_chunk(chunk)  # send chunk to keyword detector
                    if keyword_result == 0:
                        rospy.loginfo("SpeechReco:MicrophoneStream: Robot Name Keyword Detected <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
                except queue.Empty:
                    break


    def generator(self):
        self.start_time = time.time()
        while ( not self.is_keyword_mode() and not self.closed 
            and not self.interrupt_check() and not self.time_expired() ):
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            chunk = self.buff.get()
            data = None
            if chunk is None or self.interrupt_check() or self.time_expired() or self.is_keyword_mode():
                print("")
                rospy.loginfo("SpeechReco:MicrophoneStream:Generator: chunk exit ")
                return

            data = [chunk]  # for streaming recognition

            # Now consume whatever other data's still buffered.
            while not self.interrupt_check() and not self.time_expired():
                try:
                    chunk = self.buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)
            



