#! /usr/bin/env python3
# Direct Voice speaking interface, does NOT use actionlib server!

import os

class Voice(object):

    def __init__(self, audioFolder):
        self._audioFolder = audioFolder
        self.isPlaying = False
        self.isAfterPlaying = False

    def speak(self, text):
        voice = 'Linda'
        talking_speed = 0.80  # percent

        # print('DBG: EB Saying: [%s]' % (text))

        # Trap any quotes or other bad characters within the string (they mess up the call to swift)
        PERMITTED_CHARS = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ:-_., \n"
        clean_text = "".join(c for c in text if c in PERMITTED_CHARS)
        # print('DBG: Clean : [%s]' % (clean_text))

        # mute the microphone, so the robot does not talk to itself!
        ## self.mic_system_enable_pub.publish(False)
        self.isPlaying = True
        self.isAfterPlaying = False

        # Cepstral Swift is used to speak with a nice voice
        # padsp is the bridge between old OSS audio and ALSA on Linux
        # TODO - see if run_comamnd or os.system return a status, and set success appropriately

        run_command = 'padsp swift -n \'{}\' \"<prosody rate=\'{:.2f}\'>{} </prosody>\"'.format(voice, talking_speed,
                                                                                                clean_text)
        # Command line version: swift "<voice name='Linda'> <prosody rate='.8'> hello. </prosody> </voice>"

        # print('DBG: Speech Command: [%s]' % (run_command))
        os.system(run_command)

        # After speaking completes, un-mute the microphone
        self.isPlaying = False
        self.isAfterPlaying = True

    def isSpeaking(self):
        return self.isPlaying

    def isAfterSpeaking(self):
        return self.isAfterPlaying

