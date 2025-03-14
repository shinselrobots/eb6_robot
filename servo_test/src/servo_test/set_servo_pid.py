#!/usr/bin/env python3

import sys
import roslib
# roslib.load_manifest('eb_servos')
import rospy, time
from dynamixel_controllers.srv import SetPGain, SetIGain, SetDGain
from eb_servos.servo_joint_list import all_servo_joints, head_joints, right_leg_joints, left_leg_joints


class SetServoPGain():
    def __init__(self, p_gain, joints):
        set_p_gain_services = list()
        
        print("DBG: SetServoPGain: Setting p_gain values to %d" % (p_gain))
        for joint in sorted(joints):            
            print('     /' + joint)
            set_p_gain_service = '/' + joint + '/set_p_gain'
            rospy.wait_for_service(set_p_gain_service)  
            set_p_gain_services.append(rospy.ServiceProxy(set_p_gain_service, SetPGain))

        # Set the PID to requested values
        for set_p_gain in set_p_gain_services:
            set_p_gain(int(p_gain))


class SetServoIGain():
    def __init__(self, i_gain, joints):
        set_i_gain_services = list()
        print("DBG: SetServoIGain: Setting i_gain values to %d" % (i_gain))
        for joint in sorted(joints):            
            print('     /' + joint)
            set_i_gain_service = '/' + joint + '/set_i_gain'
            rospy.wait_for_service(set_i_gain_service)  
            set_i_gain_services.append(rospy.ServiceProxy(set_i_gain_service, SetIGain))

        # Set the PID to requested values
        for set_i_gain in set_i_gain_services:
            set_i_gain(int(i_gain))


class SetServoDGain():
    def __init__(self, d_gain, joints):
        set_d_gain_services = list()
        print("DBG: SetServoDGain: Setting d_gain values to %d" % (d_gain))
        for joint in sorted(joints):            
            print('     /' + joint)
            set_d_gain_service = '/' + joint + '/set_d_gain'
            rospy.wait_for_service(set_d_gain_service)  
            set_d_gain_services.append(rospy.ServiceProxy(set_d_gain_service, SetDGain))

        # Set the PID to requested values
        for set_d_gain in set_d_gain_services:
            set_d_gain(int(d_gain))





class SetServoPid():
    def __init__(self, p_gain, i_gain, d_gain, joints):

        print('SetServoPid to P = %d, I = %d, D = %d, ' %(p_gain, i_gain, d_gain))
        rospy.loginfo('SetServoPid to P = %d, I = %d, D = %d, ' %(p_gain, i_gain, d_gain))
        print(" ------------------------------------------------------------")

        SetServoPGain(p_gain, joints)
        SetServoIGain(i_gain, joints)
        SetServoDGain(d_gain, joints)

        print(" ------------------------------------------------------------")
        print("  SetServoPid complete.")


class SetSingleServoPid():
    def __init__(self, p_gain, i_gain, d_gain, servo_joint):
        # input: a servo joint string, for example: 'right_leg_knee_bend_joint'

        rospy.loginfo('SetSingleServoPid [%s] to P = %d, I = %d, D = %d, ' %(servo_joint, p_gain, i_gain, d_gain))
        #print('SetSingleServoPid [%s] to P = %d, I = %d, D = %d, ' %(servo_joint, p_gain, i_gain, d_gain))

        joints = []
        joints.append(servo_joint)
        
        try:
            SetServoPid(p_gain, i_gain, d_gain, joints)
            rospy.loginfo("*** SetSingleServoPid Done ***")
        except rospy.ROSInterruptException:
            rospy.loginfo("Oops! Exception occurred while trying to set single servo pid.") 

        print("  SetSingleServoPid complete.")



if __name__=='__main__':

    total = len(sys.argv)
    cmdargs = str(sys.argv)
    joints = all_servo_joints
    single_servo = False

    if total > 4:
        # assure P, I, D, and a servo specified

    
        option = sys.argv[1].lower()
        p_gain = int(sys.argv[2])
        i_gain = int(sys.argv[3])
        d_gain = int(sys.argv[4])

        if "all_servo_joints" in option:
            print( 'Setting all_servo_joints')
            joints = all_servo_joints
        elif "head_joints" in option:
            print( 'Setting head_joints')
            joints = head_joints
        elif "all_leg_joints" in option:
            print( 'Setting all_leg_joints')
            joints = all_leg_joints
        elif "right_leg_joints" in option:
            print( 'Setting right_leg_joints')
            joints = right_leg_joints
        elif "left_leg_joints" in option:
            print( 'Setting left_leg_joints')
            joints = left_leg_joints
        else:
            # Assume it's a specific servo
            joints = option   # + '_controller'
            print("    ===> Assuming single servo [%s].  If bad name, this will hang.  Ctrl-C to exit." %joints)
            single_servo = True

        if(single_servo):
            try:
                SetSingleServoPid(p_gain, i_gain, d_gain, joints)
                rospy.loginfo("*** Set Single Servo pid Done ***")
            except rospy.ROSInterruptException:
                rospy.loginfo("Oops! Exception occurred while trying to set single servo pid.") 

        else:
            try:
                SetServoPid(p_gain, i_gain, d_gain, joints)
                rospy.loginfo("*** Set Servo pid Done ***")
            except rospy.ROSInterruptException:
                rospy.loginfo("Oops! Exception occurred while trying to set servo pid.") 


    else:
        print( 'USAGE: set_servo_pid.py <joint_group> <p_gain> <i_gain> <d_gain> where gains are integers and group is one of: all_servo_joints, head_joints, right_leg_joints, left_leg_joints or single servo name such as head_pan_joint')
        #sys.exit()



