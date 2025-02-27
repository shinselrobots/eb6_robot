# Utility to find the correct microphone the robot needs
 
import pyaudio
import rospy
#import logging

#from ai_speak import Speech
#import ai_globals

MICROPHONE_NAME = "cvl-2005" # Name of USB mic used by robot

def find_microphone():
    print("***************** Find Microphone *****************")
    print("Opening Audio Interface pyaudio.PyAudio()")
    print("Note: you can usually ignore these ALSA messages:")
    _audio_interface = pyaudio.PyAudio()
    # _speech = Speech("audio")
    device_index = -1

    print("***************** SEARCH FOR MIC *****************")
    print(f"\nLooking for microphone...")
    # print("DBG Calling get_device_count...")
    for i in range(_audio_interface.get_device_count()):
        # print("DBG Calling get_device_info_by_index for device: ", i)
        info = _audio_interface.get_device_info_by_index(i)
        # print(f"Found device {info}")

        # print("DBG checking maxInputChannels")
        if info.get('maxInputChannels') >= 0:
            # print(f"Found a Microphone: {info}")

            if MICROPHONE_NAME in info['name'].lower():
                device_index = i
                print("")
                print(f"Device Info: {info}")
                print("")
                rospy.loginfo("Find Microphone: ========================================")
                rospy.loginfo("Find Microphone:         FOUND EB MICROPHONE!")
                rospy.loginfo("Find Microphone: ========================================")
                break

    if device_index < 0:
        # Print error / debug info if we did not find the microphone
        # _speech.speak("I have an error with my microphone subsystem.")
        print("")
        print("")
        rospy.logwarn("Find Microphone:  COULD NOT FIND REQUIRED MICROPHONE DEVICE!")
        rospy.logwarn("Find Microphone:  MAKE SURE SETTINGS --> SOUND IS *NOT OPEN*, it grabs the mic!")
        rospy.logwarn("Find Microphone:  And make sure device not in use by another ROS module!")
        print("")
        print("These other Audio Devices were found:")
        for i in range(_audio_interface.get_device_count()):
            info = _audio_interface.get_device_info_by_index(i)
            print(f"  {info}")
        rospy.logwarn("================= WARNING: SPEECH RECOGNITION WILL NOT WORK! =========================")
        print("")

    return device_index
