#!/usr/bin/env python3
# Servo status client using joint_states_listener

import roslib
import rospy
from eb_servos.srv import ReturnJointStates
import time
import sys
import math
# -------------------------------------------------------------------------
from servo_joint_list import *


def display_joint_states(joint_names):
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
        else:
            flag_string = ''
            if math.fabs(resp.effort[ind]) > .1:
                flag_string = ' <----'
            elif resp.effort[ind] == 0.0:
                flag_string = ' <<<<<'

            if(ind == 6 or ind == 9):
                print("")
                
            #print("%s = \t %-2.3f   %s" % (joint_name, resp.effort[ind], flag_string)) 
            print('{0:7d} {1:30s} {2: 2.3f} {3: 2.2f} {4: 2.1f} {5:s}'.format(
                ind+1, joint_name, resp.position[ind], resp.effort[ind], resp.temp[ind], flag_string))
             
            #print(f"{joint_name} = \t {resp.effort[ind]}")
    return (resp.position, resp.velocity, resp.effort)


#pretty-print list to string
def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])


#print out the positions, velocities, and efforts of the joints
if __name__ == "__main__":

    while(1):
        print("")
        print('      --------------------------------  Radian Load  Temp C')
        display_joint_states(all_servo_joints)
        time.sleep(1)
