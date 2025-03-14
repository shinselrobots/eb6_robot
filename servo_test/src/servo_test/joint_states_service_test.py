#!/usr/bin/env python3
# test client for joint_states_listener

import roslib
import rospy
from eb_servos.srv import ReturnJointStates
import time
import sys
# -------------------------------------------------------------------------
from servo_joint_list import *


def call_return_joint_states(joint_names):
    rospy.wait_for_service("return_joint_states")
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException as e:
        print("error when calling return_joint_states: %s" %e)
        sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print("joint %s not found!"%joint_name)
    #return (resp.position, resp.velocity, resp.effort)
    return (resp.position, resp.velocity, resp.effort, resp.goal_pos, resp.error)


#pretty-print list to string
def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])


#print out the positions, velocities, and efforts of the joints
if __name__ == "__main__":

    rospy.init_node('joint_states_service_test')
    start_time = rospy.get_rostime()
    
    prior = []
    last_pos = []
    
    while(1):
        #   print('------------------------------')
        # select joints to test:  right_leg_joints, all_servo_joints, etc.
        # (position, velocity, effort) = \
        (position, velocity, effort, goal, error) = \
            call_return_joint_states(all_servo_joints)
        #print("position:", pplist(position))
        #print("velocity:", pplist(velocity))
        #print("effort:", pplist(effort))
        #print("  last:", pplist(effort))
        #print("goal_pos:", pplist(goal))
        #print("error:", pplist(error))

 
        # see how quickly we get actual new data (report rate from dynamixel_driver)
        if not prior:
            print("first one")
        else:
            if not prior == position:
                stop_time =  rospy.Time.now()       
                elapsed_time = stop_time - start_time
                elapsed_sec = elapsed_time.to_sec()
                start_time = stop_time
                print("Position changed. Time = %08f ms, %04.6f sec." % ((elapsed_sec * 1000), elapsed_sec) )
                #print("position changed")
                #print("position:", pplist(position))
                
        prior = position
        
        #time.sleep(0.1)
