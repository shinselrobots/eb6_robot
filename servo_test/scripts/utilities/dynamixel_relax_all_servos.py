#!/usr/bin/env python3

"""
    relax_all_servos.py - Version 0.1 2012-03-24
    
    Relax all servos by disabling the torque and setting the speed
    and torque limit to a moderate values.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html

    NOTE: Modified 8/2017, Dave Shinsel

"""
import roslib
# roslib.load_manifest('eb_servos')
import rospy, time
from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed
from servo_joint_list import all_servo_joints, head_joints, right_arm_joints

class Relax():
    def __init__(self):
        rospy.init_node('dynamixel_relax_all_servos')
                
        default_dynamixel_speed = rospy.get_param('~default_dynamixel_speed', 0.5)
        default_dynamixel_torque = rospy.get_param('~default_dynamixel_torque', 0.5)

        speed_services = list()   
        torque_services = list()
        set_torque_limit_services = list()

        print('starting loop...')
            
        for controller in sorted(all_servo_joints):            
            torque_service = '/' + controller + '/torque_enable'
            print('  waiting for service: ' + torque_service)

            rospy.wait_for_service(torque_service)  
            torque_services.append(rospy.ServiceProxy(torque_service, TorqueEnable))
            
            set_torque_limit_service = '/' + controller + '/set_torque_limit'
            rospy.wait_for_service(set_torque_limit_service)  
            set_torque_limit_services.append(rospy.ServiceProxy(set_torque_limit_service, SetTorqueLimit))
            
            speed_service = '/' + controller + '/set_speed'
            rospy.wait_for_service(speed_service)  
            speed_services.append(rospy.ServiceProxy(speed_service, SetSpeed))
        
        # Set the default speed to something small
        print('  setting startup speeds to ', default_dynamixel_speed)
        for set_speed in speed_services:
            try:
                set_speed(default_dynamixel_speed)
            except:
                pass
            
        # Set the torque limit to a moderate value
        print('  setting torque limits to ', default_dynamixel_torque)
        for set_torque_limit in set_torque_limit_services:
            try:
                set_torque_limit(default_dynamixel_torque)
            except:
                pass

        # Relax all servos to give them a rest.
        for torque_enable in torque_services:
            try:
                torque_enable(False)
            except:
                pass

        print("Done relaxing servos.")

        
if __name__=='__main__':
    Relax()
