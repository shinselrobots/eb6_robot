#! /usr/bin/env python

import rospy
import actionlib
import sys
import robot_voice.msg

def speech_client(text_to_speak=''):
    # Creates the SimpleActionClient, passing the type of the action
    # to the constructor.
    client = actionlib.SimpleActionClient('/speech_service', robot_voice.msg.speechAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo('Waiting for server...')   
    client.wait_for_server()

    # Creates a goal to send to the action server.
    if text_to_speak == '':
        text_to_speak='hello, my name is E B Six.  I am your loyal robot companion'
    
    goal = robot_voice.msg.speechGoal(text_to_speak)

    # Sends the goal to the action server.
    rospy.loginfo('Sending Goal')   
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    rospy.loginfo('Waiting for result...')   
    client.wait_for_result()

    # Prints out the result of executing the action
    rospy.loginfo('Done with Goal')   
    return client.get_result()  

if __name__ == '__main__':

    total_args = len(sys.argv)
    cmdargs = str(sys.argv)
    text_to_speak = ""
    if total_args > 1:
        text_to_speak = sys.argv[1]
        print("Text to speak = ", text_to_speak)
        
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('speech_test_client_py')
        rospy.loginfo('Starting robot_voice %s action client!', rospy.get_name() )   
        result = speech_client(text_to_speak)
        print('Done Talking.' )

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
