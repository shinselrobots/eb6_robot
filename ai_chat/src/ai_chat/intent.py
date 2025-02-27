#!/usr/bin/env python3
# Receives raw text (from AI or speech to text) and performs a very simple intent extraction.
# If valid command, sends to the behavior server 

import rospy
import os
import sys
import time
import signal
import yaml


from std_msgs.msg import Bool
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32

from behavior_msgs.msg import CommandState


# For DEBUGGING from the command line
DEBUG_LOAD_LOCAL_YAML = False
DEBUG_YAML_PATH = '../../config/intent.yaml'


class CommandIntent():
    def __init__(self):
        ##rospy.init_node('command_intent')
        self.logname = 'AI Command Intent: '
        rospy.loginfo(self.logname + "initializing...")

        # Load the configuration yaml file into a dictionary
        
        self.commandDict = None
        rospy.loginfo(self.logname + "Opening YAML Config file.")
        
        if DEBUG_LOAD_LOCAL_YAML:
            # For debug only, allows running from command line without launch file
            print("DEBUG: Loading local YAML")
            # If this fails, it will throw an exception, killing the process immediately (which we want)
            with open(DEBUG_YAML_PATH, 'r') as config_file:
                self.commandDict = yaml.load(config_file, Loader=yaml.SafeLoader)
            rospy.loginfo(self.logname + "YAML Config file Opened.")

        else:
            print("Loading YAML from launch file")
            self.commandDict = rospy.get_param('ai_chat', None)


            
        # DEBUG Print the yaml values as a dictionary
        print(self.commandDict)
        print()

        ## self.fake_name = 'none'
        self.last_name = ''
        self.last_name_loop_count = 0
        ##self.got_name_time = datetime.now()
        self.name_timeout = 2000 # miliseconds
        self.ai_enabled = True # Keep track of AI state

        # SUBSCRIBERS
        
        ##person_detected_topic = "/body_tracker_array/people"  # Message with info on array of detected people
        ## rospy.get_param("~person_detection_topic", "/body_tracker_array/people")
        ## rospy.loginfo("Intent Extration: Listening on person detection Topic: " + person_detected_topic)
        # TODO: self.sub_detection = message_filters.Subscriber(person_detected_topic, BodyTrackerArray)
        ##rospy.Subscriber(person_detected_topic, BodyTrackerArray, self.person_detection_callback)

        # Receive recognized text from speech recognition node
        ##s = rospy.Service('speech_handler', speech_recognition_intent, self.speechCallback)


        # PUBLISHERS
        # Publish an action for the behavior engine to handle
        self.behavior_cmd_pub = rospy.Publisher('behavior/cmd', CommandState, queue_size=2)
        self.ai_enable_pub = rospy.Publisher('/ai_enable',   Bool, queue_size=4)

        rospy.loginfo(self.logname + "service available")




    def logConfiguration(self):
        # Pretty print out the loaded configuration data
        for command in self.commandDict['commands']:
            rospy.loginfo( "Command for: '%s'", command['description'])
            rospy.loginfo( "  Required words(s):")
            for word in command['required_word_lists']:
                rospy.loginfo ("    %s", word)
            rospy.loginfo("  intent: %s", command['intent'])
            if 'param1' in command:
                rospy.loginfo("  param1: %s", command['param1'])
            if 'param2' in command:
                rospy.loginfo("  param2: %s", command['param2'])


    def detect_command(self, ai_response_message):
        cmd_string = ''
        non_cmd_string = ''
        end_delimiter = ''
        minimum_delimiter = 1000
        ai_response_message_lower = ai_response_message.lower()

        #print("parsing AI string: ", ai_response_message_lower)
        for start_delimiter in '[(*':
            #print("start_delimiter = ", start_delimiter)
            if start_delimiter == '[':
                end_delimiter = ']'
            elif start_delimiter == '(':
                end_delimiter = ')'
            else:
                end_delimiter = '*'

            cmd_start = ai_response_message_lower.find(start_delimiter)
            if cmd_start >= 0:
                cmd_end = ai_response_message_lower.find(end_delimiter, cmd_start + 1)
                if cmd_end >= 0:
                    # Save minimum delimiter, so we can cut all cmd strings out of the response (stupid AI!)
                    if cmd_start < minimum_delimiter:
                        minimum_delimiter = cmd_start
                        # Save the cmd in order of preferred delimiters
                    if cmd_string == '':
                        cmd_string = ai_response_message_lower[cmd_start + 1: cmd_end]
                        print("*******************************************************")
                        print("AI:Intent: AI found a COMMAND! [%s]" % cmd_string)

        if cmd_string != '':
            non_cmd_string = ai_response_message_lower[:minimum_delimiter]
            print("DBG AI:Intent: non-cmd string: [%s] " % non_cmd_string)
        else:
            print("DBG AI:Intent: No command found")

        return cmd_string, non_cmd_string




    def parseCommandHeard(self, heard_phrase):
        # For a given raw string input, walk through each of the 
        # intent commands and see if there is a match for it.  
        #
        # Note: Currently matches 'joke' in "jokes".  need better
        # substring checker. In the meantime order your speech carefully!

        #print("")
        #print("***********************************************************************************************")
        #print("DBG: Intent:ParseCommand DEBUG:")
        for command in self.commandDict['commands']:
            #print("command = ", command)

            required_phrase_found = False
            for required_phrase in command['required_word_lists']:
                ##print("required_phrase = [%s]" % required_phrase)
                
                for required_word in required_phrase.split():
                    ##print("required_word = [%s]", required_word)
                    required_word_found = False
                    
                    for heard_word in heard_phrase.split():
                        ##print("heard_word = [%s], required_word = [%s]" % (heard_word, required_word))
                        if required_word == heard_word:
                            print("Speech Handler:Intent:ParseCommand: required_word [%s] found." % required_word)
                            required_word_found = True
                            break
                    if required_word_found:
                        ##print("required word found in phrase")
                        continue
                    else:
                        ##print("required word not found in phrase")
                        break
                if required_word_found:
                    # all required words found in phrase!
                    required_phrase_found = True
                    print("Speech Handler:Intent:ParseCommand: All required words found! [%s] Command confirmed!" % required_phrase)
                    break
                ##else:
                    ##print("required words not found in phrase [%s], trying next phrase" % required_phrase)
            if required_phrase_found:
                rospy.loginfo("Speech Intent: got command [%s]", command)               
                return command

        return None


    def handle_command(self, command_phrase):

        command_phrase = command_phrase.lower()
        print("%s: Command Phrase = [%s]" % (self.logname, command_phrase))

        if command_phrase == '':
            return False # no command found       

        command = self.parseCommandHeard(command_phrase)
        if None == command:
            rospy.loginfo(self.logname + "No valid command detected.")
            return False # no command found        
                
        else:
            rospy.loginfo(self.logname + "command recognized")

            # Send intent to the behavior server
            msg = CommandState()
            msg.commandState = command['intent']
            if 'param1' in command:
                msg.param1 = str(command['param1'])
            if 'param2' in command:
                msg.param2 = str(command['param2'])
            rospy.loginfo("%s: Sending Behavior Command to /behavior/cmd.  CMD: %s, PARAM1: %s, PARAM2: %s" %
                (self.logname, msg.commandState, msg.param1, msg.param2))
                
            ##if DEBUG_ENABLE_COMMAND_SEND:                
            self.behavior_cmd_pub.publish(msg)

        return True # command found        


if __name__ == '__main__':
    #  JUST USED FOR TESTING
    rospy.init_node('intent_test')
    test = CommandIntent() # requires DEBUG_LOAD_LOCAL_YAML = True
    test.handle_command("Turn Right")
    test.handle_command("Turn Left")
    test.handle_command("Move forward")
    test.handle_command("FOOBAR")
    
    

