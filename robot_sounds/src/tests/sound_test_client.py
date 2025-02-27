#! /usr/bin/env python

import rospy
import actionlib
import robot_sounds.msg

def sound_test_client():
    # Creates the SimpleActionClient, passing the type of the action
    # to the constructor.
    client = actionlib.SimpleActionClient('sound_service', robot_sounds.msg.soundAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo('Waiting for server...')   
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = robot_sounds.msg.soundGoal(filename_to_play='/home/system/test.wav')

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
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('sound_test_client_py')
        rospy.loginfo('Starting robot_sounds %s action client!', rospy.get_name() )   
        result = sound_test_client()
        print('Done Playing.' )

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)

