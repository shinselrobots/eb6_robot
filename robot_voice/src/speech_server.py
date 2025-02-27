#! /usr/bin/env python

# Robot voice Text to speech server. Will work with any os-invoked TTS.
# Currently supports:
# Cepstral(Older) https://www.cepstral.com/
# Piper TTS (Newer) https://github.com/rhasspy/piper

import rospy
import actionlib
import robot_voice.msg
import os
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import UInt32
from system_status_msgs.msg import SystemStatus

DEFAULT_RESOURCE_DIR = '/home/system/catkin_robot/resources/'


class SystemStatusUpdate():
    def __init__(self):
        self.status_pub = rospy.Publisher('/system_status', SystemStatus, queue_size=8)

    def send_status_update(self, item, status):
        # Send status update to system monitor
        status_msg = SystemStatus()
        status_msg.item = item
        status_msg.status = status
        self.status_pub.publish(status_msg)


class speechAction(object):
    # create messages that are used to publish feedback/result
    _feedback = robot_voice.msg.speechFeedback()
    _result = robot_voice.msg.speechResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, 
            robot_voice.msg.speechAction, execute_cb=self.execute_cb, auto_start = False)

        self.send_status_update = SystemStatusUpdate().send_status_update
        
        # Default voice options
        self.tts_engine = 'piper'  # cepstral or piper
        #self.voice_name = 'linda'  # Cepstral default voice for EB6
        #self.voice_name = 'lessac'  # piper default voice for EB6
        self.voice_name = 'poppy'  # piper default voice for EB6

        self.resource_dir = rospy.get_param('resource_dir', DEFAULT_RESOURCE_DIR)
        self.abs_path_to_piper = os.path.join(DEFAULT_RESOURCE_DIR, 'sdk/piper')
        #OLD self.abs_path_to_piper = '/home/system/sdk/piper'
        #print("DEBUG: abs_path_to_piper = [%s]" % (self.abs_path_to_piper))
        
        rel_path_to_voice = ''
        self. speaker_id_string = ''
        self.path_to_voice = ''
        self.debug_voice_number = 0
        
        # initialize the default voice
        self.set_voice_name(self.voice_name)

        # PUBLISHERS
        # enable/disable microphone when robot is talking or moving servos.  
        # (Note system_enable vs. user_enable)
        #self.mic_system_enable_pub = rospy.Publisher('/microphone/system_enable', Bool, queue_size=1)        

        # SUBSCRIBERS
        rospy.Subscriber("/voice_name", String, self.voice_name_callback)
        rospy.Subscriber("/voice_number", UInt32, self.voice_number_callback) # for testing voices with multiple speakers

        self.send_status_update("VOICE", "READY")
        self._as.start()

    def set_voice_name(self, name):
        voice_name = name.lower() 

        if voice_name == 'linda' or voice_name == 'david':
            self.tts_engine = 'cepstral'
            self.voice_name = voice_name

        elif (voice_name == 'poppy'  or voice_name == 'cori' or
            voice_name == 'prudence' or voice_name == 'lessac' or
            voice_name == 'spike' or voice_name == 'libritts'):
        
            self.tts_engine = 'piper'
            self.voice_name = voice_name
            self.speaker_id_string = ''
            
            if voice_name == 'poppy':
                rel_path_to_voice = '/voices/semaine/en_GB-semaine-medium.onnx'
                self.speaker_id_string = ' -s 3' # poppy's voice

            elif voice_name == 'prudence':
                rel_path_to_voice = '/voices/semaine/en_GB-semaine-medium.onnx'
                self.speaker_id_string = ' -s 0' # prudence's voice

            elif voice_name == 'spike':
                rel_path_to_voice = '/voices/semaine/en_GB-semaine-medium.onnx'
                self.speaker_id_string = ' -s 1' # spike's voice

            elif voice_name == 'cori':
                rel_path_to_voice = '/voices/cori/en_GB-cori-high.onnx'
                self.speaker_id_string = '' # only one voice in the model

            elif voice_name == 'lessac':
                rel_path_to_voice = '/voices/lessac/en_US-lessac-medium.onnx'
                self.speaker_id_string = '' # only one voice in the model

            elif voice_name == 'libritts':
                rel_path_to_voice = '/voices/libritts/en_US-libritts-high.onnx'
                self.speaker_id_string = ' -s 0' # first voice of 900!

            else:
                rospy.logwarn('%s: WARNING Voice Name [%s] logic error. ' % (self._action_name, voice_name))
                rel_path_to_voice = '/voices/semaine/en_GB-semaine-medium.onnx' # use poppy as default
                self.speaker_id_string = ' -s 3' # poppy's voice

            self.path_to_voice = self.abs_path_to_piper + rel_path_to_voice


        else:
            rospy.logwarn('%s: WARNING Bad Voice Name [%s]. Ignored.' % (self._action_name, voice_name))



    def voice_name_callback(self, data):
                
        voice_name = data.data
        rospy.loginfo('%s: New voice [%s] requested.' % (self._action_name, voice_name))
        self.set_voice_name(voice_name)

    def voice_number_callback(self, data):
        # For testing certain voices that support multiple "speakers"                
        voice_number = data.data
        rospy.loginfo('%s: New voice number [%d] requested.' % (self._action_name, voice_number))
        # WARNING! may screw up some voices! if self.voice_name == 'libritts':
        #rospy.loginfo('%s: Current voice [%s] supports numbers. Setting speaker to: %d.' % 
        rospy.loginfo('%s: Current voice [%s].  Setting speaker to: %d.' % 
            (self._action_name, self.voice_name, voice_number))
        self.speaker_id_string = ' --s ' + str(voice_number) 

      
    def execute_cb(self, goal):
        # goal.text has the text to speak
        # helper variables
        success = True
        rospy.loginfo('%s: Voice [%s] is saying: [%s]' % (self._action_name, self.voice_name, goal.text_to_speak))

        # mute the microphone, so the robot does not talk to itself!
        #self.mic_system_enable_pub.publish(False)

        # Trap any quotes or other bad characters within the string (they mess up the call to swift)
        PERMITTED_CHARS = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ:-_., \n"
        clean_text = "".join(c for c in goal.text_to_speak if c in PERMITTED_CHARS)
        rospy.loginfo('%s: Clean : [%s]' % (self._action_name, clean_text))

        if False: # DEBUG TODO REMOVE THIS
            run_command =  "echo 'Hello' | /home/system/sdk/piper/piper --model /home/system/sdk/piper/voices/semaine/en_GB-semaine-medium.onnx -s 3 --output_raw | aplay -f S16_LE -c1 -r22050"

        else:
            self.send_status_update("VOICE", "SPEAKING")
        
            if self.tts_engine == 'cepstral':
                # use Cepstral TTS

                talking_speed = 1.00 # percent
                if self.voice_name == 'linda':
                    talking_speed = 0.90 # percent
                
                # default volume is "medium" Others are: 'loud', 'x-loud'. 
                # See: https://www.cepstral.com/en/tutorials/view/ssml
                talking_volume = 'medium' 

                # Cepstral Swift is used to speak
                # padsp is the bridge between old OSS audio and ALSA on Linux
                # TODO - see if os.system returns a status, and set success appropriately
                
                run_command = 'padsp swift -n \'{}\' \"<prosody rate=\'{:.2f}\'>{} </prosody>\"'.format( 
                    self.voice_name, talking_speed, clean_text )
                # Cmd Line: padsp swift "<voice name='linda'> <prosody rate='.8'> hello. </prosody> </voice>"


            else: # use default engine,  piper tts
                
                run_command = ("echo '" + clean_text + "' | " + self.abs_path_to_piper + '/piper --model ' +  self.path_to_voice + 
                    self.speaker_id_string + ' --output_raw | aplay -f S16_LE -c1 -r22050')

        
        rospy.loginfo('%s: Speech Command: [%s]' % (self._action_name, run_command))
        os.system(run_command)

        # note, no way to interrupt this (yet) so preemption ignored.
        #    if self._as.is_preempt_requested():
        #        rospy.loginfo('%s: Preempted' % self._action_name)
        #        self._as.set_preempted()
        #        success = False
        #        break
        #    self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
        #    self._as.publish_feedback(self._feedback)

        # After speaking completes, un-mute the microphone
        #self.mic_system_enable_pub.publish(True)
        self.send_status_update("VOICE", "READY")
          
        if success:
            self._result.complete = True
            rospy.loginfo('%s: Done Talking' % self._action_name)
            print()
            print()
            self._as.set_succeeded(self._result)
 
 
 
       
if __name__ == '__main__':
    rospy.init_node('speech_service')
    rospy.loginfo('Starting robot_voice %s action server!', rospy.get_name() )   
    server = speechAction(rospy.get_name())
    rospy.spin()
