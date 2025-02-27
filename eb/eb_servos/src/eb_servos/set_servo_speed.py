#!/usr/bin/env python3
# Sets speed in RADIANS / SEC
# for example, the MX-28 max Rad/sec is 5.0

import sys
import roslib
# roslib.load_manifest('eb_servos')
import rospy, time
from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed
from eb_servos.servo_joint_list import *

# speed value in Radians/Second. 
# MX28:   55 RPM --> 5.8 rad/sec (2.5 NM Torque)
# MX64:   63 RPM --> 6.6 rad/sec (6.0 NM Torque)
# MX106:  35 RPM --> 3.7 rad/sec (8.4 NM Torque)
# For our purposes, we usually assume max speed of 5.0, so all servos run at predictable speed.

class SetServoSpeed():
    def __init__(self, speed, joints):
        # rospy.loginfo("DBG: SetServoSpeed: setting to %1.4f rad/sec:" %speed)
        # rospy.loginfo("DBG: Servo Speed (press ctrl-c to cancel at anytime)")

        speed_services = list()
        
        # print("DBG: Joints List = ", joints)

        for controller in sorted(joints):            
            speed_service = '/' + controller + '/set_speed'
            # print('  ' + speed_service)
            # print("DBG: Waiting for service...") # IF THIS HANGS, check joint name is 'foo_joint' and servos are launched
            rospy.wait_for_service(speed_service)
            # print("DBG: done waiting. Service ready")  
            speed_services.append(rospy.ServiceProxy(speed_service, SetSpeed))
            
        # Set the speed
        # print( '  setting servo speeds to ', speed)
        for set_speed in speed_services:
            set_speed(speed)

        # print("DBG: SetServoSpeed complete.")

        
class SetSingleServoSpeed():
    def __init__(self, speed, servo_joint):
        # input: a servo controller string, for example: 'right_leg_shoulder_rotate_joint'

        #rospy.loginfo('DBG: SetSingleServoSpeed [%s] to %1.4f' % (servo_joint, speed))

        joints = []
        joints.append(servo_joint)
        #print("DBG: SetSingleServoSpeed joints = ", joints)  
             
        try:
            SetServoSpeed(speed, joints)
            #rospy.loginfo("DBG: Set Single Speed Done")
        except rospy.ROSInterruptException:
            rospy.loginfo("SetServoSpeed: Oops! Exception occurred while trying to set single speed.") 

        # print("DBG: SetSingleServoSpeed complete.")

        
if __name__=='__main__':

    total = len(sys.argv)
    cmdargs = str(sys.argv)
    joints = all_servo_joints
    single_servo = False

    if total > 2:
        option = sys.argv[1].lower()
        speed = float(sys.argv[2])
        
        if "all_servo_joints" in option:
            print( 'Setting all_servo_joints')
            joints = all_servo_joints
        elif "head_joints" in option:
            print( 'Setting head_joints')
            joints = head_joints
        elif "right_leg_joints" in option:
            print( 'Setting right_leg_joints')
            joints = right_leg_joints
        elif "left_leg_joints" in option:
            print( 'Setting left_leg_joints')
            joints = left_leg_joints
        else:
            # Assume it's a specific servo
            joints = option  
            print("    ===> Assuming single servo [%s].  If bad name, this will hang.  Ctrl-C to exit." %joints)
            single_servo = True

        if(single_servo):
            try:
                SetSingleServoSpeed(speed, joints)
                # rospy.loginfo("*** Set Single Servo Speed Done ***")
            except rospy.ROSInterruptException:
                rospy.loginfo("SetServoSpeed: Oops! Exception occurred while trying to set single servo speed.") 

        else:
            try:
                SetServoSpeed(speed, joints)
                # rospy.loginfo("*** Set Servo Speed Done ***")
            except rospy.ROSInterruptException:
                rospy.loginfo("SetServoSpeed: Oops! Exception occurred while trying to set servo speed.") 


    else:
        print( 'USAGE: set_servo_speed.py <joint_group> <speed_value>   where group is one of: all_servo_joints, head_joints, right_leg_joints, left_leg_joints or single servo name such as head_pan_joint')
        #sys.exit()
        
        
        


