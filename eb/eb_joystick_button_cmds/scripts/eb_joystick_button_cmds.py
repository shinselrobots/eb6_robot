#!/usr/bin/env python3
# Runs in two modes: Normal Control Mode and Servo Record Mode
# Change Mode below for servo recording or normal operation
# Done here to avoid accidentally entering record mode during normal operation.
# Invokes behaviors when user presses a joystick button

import rospy
import logging
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import UInt16
from std_msgs.msg import Bool
from behavior_msgs.msg import CommandState


def callback(data):

    # Invoke behaviors when user presses a joystick button
    # we only allow one behavior state at a time
    msg = CommandState()

    for i in range(0, 11):
        if data.buttons[i] == 1: 
            rospy.loginfo("Button %d Pressed", i)

    if data.buttons[0] == 1:  # Blue X button - Pose Medium (normal)
        msg.commandState = "POSE"
        msg.param1 = "mid"
        msg.param2 = '0.3'
        pub_behavior.publish(msg) 
        rospy.loginfo("Moving to Stand Mid...")
        
    elif data.buttons[1] == 1:  # Green A button - Pose Low (sit)
        msg.commandState = "POSE"
        msg.param1 = "low"
        msg.param2 = '0.3'
        pub_behavior.publish(msg) 
        rospy.loginfo("Moving to Stand Low...")
        
    elif data.buttons[2] == 1:    # Red B button - Stop all behaviors
        # BROKEN - Hangs the behaviors for some reason: msg.commandState = "STOP"
        # pub_behavior.publish(msg) 
        rospy.loginfo("Stop button disabled")
        
    elif data.buttons[3] == 1:  # Yellow Y button - Pose High
        msg.commandState = "POSE"
        msg.param1 = "mid-high"
        msg.param2 = '0.3'
        pub_behavior.publish(msg) 
        rospy.loginfo("Moving to Stand High...")
        
    elif data.buttons[8] == 1:  # "Back" button - Sleep
        msg.commandState = "SLEEP"
        pub_behavior.publish(msg) 
        rospy.loginfo("Invoking Sleep...")
        
    elif data.buttons[9] == 1:  # "Start" button - Wakeup
        msg.commandState = "WAKEUP"
        pub_behavior.publish(msg) 
        rospy.loginfo("Ivoking Wakeup...")

    elif data.buttons[7] == 1:  # "Right bottom trigger - use for testing stuff
        pub_mark_time.publish(1)  # just publish a marker



def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.loginfo("Joy_buttons Ready:")
    rospy.loginfo("   A = Stand Low")
    rospy.loginfo("   X = Stand Mid")
    rospy.loginfo("   Y = Stand HIgh")
    rospy.loginfo("   B = Stop Behaviors")
    rospy.loginfo("   Back = Sleep")
    rospy.loginfo("   Start = Wakeup")

    rospy.spin()



if __name__ == '__main__':

    #print("Starting Joy Buttons!")
    rospy.loginfo("Starting Joy Buttons")

    # Publish an action for the behavior engine to handle
    pub_behavior = rospy.Publisher('behavior/cmd', CommandState, queue_size=2)
    pub_mark_time = rospy.Publisher('/user_mark_time', UInt16, queue_size=1)
    listener()
    
    
    
