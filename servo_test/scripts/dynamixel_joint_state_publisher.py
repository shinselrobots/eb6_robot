#!/usr/bin/env python3

"""
    dynamixel_joint_state_publisher.py - Version 1.0 2010-12-28
    
    Publish the dynamixel_controller joint states on the /joint_states topic
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""


# import roslib; roslib.load_manifest('eb_servos')
import rospy

# from sensor_msgs.msg import JointState as JointStatePR2
from dynamixel_msgs.msg import JointState as JointStateDynamixel
from dynamixel_msgs.msg import JointStateArray

class JointStateMessage():
    def __init__(self, name, position, velocity, effort, goal_pos, error, temp):
        self.name = name
        self.position = position
        self.velocity = velocity
        self.effort = effort
        self.goal_pos = goal_pos
        self.error = error
        self.temp = temp
        

class JointStatePublisher():
    def __init__(self):
        rospy.init_node('dynamixel_joint_state_publisher', anonymous=True)
        
        rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(rate)
        
        # The namespace and joints parameter needs to be set by the servo controller
        # (The namespace is usually null.)
        namespace = rospy.get_namespace()
        self.joints = rospy.get_param(namespace + '/controllers', '')
                                                                
        rospy.loginfo("Dynamixel Joint State Publisher namespace =  " + namespace)

        self.servos = list()
        self.controllers = list()
        self.joint_states = dict({})
        
        for controller in sorted(self.joints):       # current_position, velocity, load, goal_pos, error, temp  
            self.joint_states[controller] = JointStateMessage(controller, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.controllers.append(controller)
                           
        # Start controller state subscribers
        [rospy.Subscriber(c + '/state', JointStateDynamixel, self.controller_state_handler) for c in self.controllers]
     
        # Start publisher
        self.joint_states_pub = rospy.Publisher('/joint_states_array', JointStateArray, queue_size=4)
       
        rospy.loginfo("Starting Dynamixel Joint State Array Publisher at " + str(rate) + "Hz")
       
        while not rospy.is_shutdown():
            self.publish_joint_states()
            r.sleep()
           
    def controller_state_handler(self, msg):
    
        js = JointStateMessage(msg.name, msg.current_pos, msg.velocity, msg.load,  msg.goal_pos, msg.error, msg.motor_temps[0])
        self.joint_states[msg.name] = js
        #print("MSG = ", msg.motor_temps)
        #temps = msg.motor_temps
        #temp = temps[0]
        #print("TEMP = ", temp)
       
    def publish_joint_states(self):
        # Construct message & publish joint states
        msg = JointStateArray()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []
        msg.goal_pos = []
        msg.error = []
        msg.temp = []
       
        for joint in self.joint_states.values():
            msg.name.append(joint.name)
            msg.position.append(joint.position)
            msg.velocity.append(joint.velocity)
            msg.effort.append(joint.effort)
            msg.goal_pos.append(joint.goal_pos)
            msg.error.append(joint.error)
            msg.temp.append(joint.temp)
            
            #print("New Msg = ", msg)
            
           
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        self.joint_states_pub.publish(msg)
        
if __name__ == '__main__':
    try:
        s = JointStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException: pass

