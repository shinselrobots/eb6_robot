#!/usr/bin/env python3

import sys
import roslib
# roslib.load_manifest('eb_servos')
import rospy, time
from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed
from eb_servos.servo_joint_list import all_servo_joints, head_joints, right_leg_joints, left_leg_joints

class SetServoTorque():
    def __init__(self, torque, joints):
                       
        rospy.loginfo('SetServoTorque to %1.4f: (1.0 = max)' %torque)

        torque_enable_services = list()
        set_torque_limit_services = list()

        for joint in sorted(joints):            
            print('  /' + joint)
            torque_enable_service = '/' + joint + '/torque_enable'
            set_torque_limit_service = '/' + joint + '/set_torque_limit'

            rospy.wait_for_service(torque_enable_service)  
            torque_enable_services.append(rospy.ServiceProxy(torque_enable_service, TorqueEnable))
            
            rospy.wait_for_service(set_torque_limit_service)  
            set_torque_limit_services.append(rospy.ServiceProxy(set_torque_limit_service, SetTorqueLimit))

        if torque == 0.0:
            # Turn off torque
            #print("  turning off torque...")
            for torque_enable in torque_enable_services:
                torque_enable(False)
            #print("  torque off")

        else:
            # Set the torque limit to a requested value
            #print '  setting torque limits to ', torque
            for set_torque_limit in set_torque_limit_services:
                set_torque_limit(torque)

            # Enable torque.
            for torque_enable in torque_enable_services:
                torque_enable(True)

        print("  SetServoTorque complete.")


class SetSingleServoTorque():
    def __init__(self, torque, servo_joint):
        # input: a servo joint string, for example: 'right_leg_knee_bend_joint'
                       
        rospy.loginfo('SetSingleServoTorque [%s] to %1.4f: (1.0 = max)' %(servo_joint, torque))
        # print('  /' + servo_joint)

        joints = []
        joints.append(servo_joint)
        
        try:
            SetServoTorque(torque, joints)
            rospy.loginfo("*** Set Single Torque Done ***")
        except rospy.ROSInterruptException:
            rospy.loginfo("Oops! Exception occurred while trying to set single torque.") 

        print("  SetSingleServoTorque complete.")



if __name__=='__main__':

    total = len(sys.argv)
    cmdargs = str(sys.argv)
    joints = all_servo_joints
    single_servo = False
    
    if total > 2:
        option = sys.argv[1].lower()
        torque = float(sys.argv[2])

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
                SetSingleServoTorque(torque, joints)
                rospy.loginfo("*** Set Single Servo Torque Done ***")
            except rospy.ROSInterruptException:
                rospy.loginfo("Oops! Exception occurred while trying to set single servo torque.") 

        else:
            try:
                SetServoTorque(torque, joints)
                rospy.loginfo("*** Set Servo Torque Done ***")
            except rospy.ROSInterruptException:
                rospy.loginfo("Oops! Exception occurred while trying to set servo torque.") 


    else:
        print( 'USAGE: set_servo_torque.py <joint_group> <torque_value>   where group is one of: all_servo_joints, head_joints, right_leg_joints, left_leg_joints or single servo name such as head_pan_joint')
        #sys.exit()



