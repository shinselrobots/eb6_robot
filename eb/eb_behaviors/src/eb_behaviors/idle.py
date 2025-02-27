#! /usr/bin/env python

import rospy
import time
#import random
#from random import randint

from math import radians, degrees
import rospkg
import rosparam

from std_msgs.msg import Bool
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Float64
from std_msgs.msg import String

import os 
import _thread
# import tf # Warning - needs recompile for Python3. See: 
#  https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/

from playsound import playsound

# for servos
#from eb_servos.set_pose import *
#from eb_servos.servo_joint_list import head_joints
#from eb_servos.head_servo_publishers import *
#from eb_servos.set_servo_speed import *
#from eb_servos.set_servo_torque import *
#from eb_servos.srv import ReturnJointStates
#from eb_servos.head_control import HeadControl

#from body_tracker_msgs.msg import BodyTracker, BodyTrackerArray
#from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose, Pose2D
#import geometry_msgs.msg

DBG_DISABLE_RANDOM_MOVEMENT = False # OVERRIDE to disable while debugging




class IdleBehavior():

    def __init__(self, behavior_utils, interrupt_check):
        self.module_name = 'idle_behavior'
        rospy.loginfo('%s: Initializing...' % (self.module_name))
        
        self.interrupt_check = interrupt_check
        self.send_status_update = behavior_utils.send_status_update
        self.speak = behavior_utils.speak
        #self.head_control = HeadControl(self.module_name)

        #self.robotpose = RobotPose(self.module_name)
        # Get this behavior's *GLOBAL* parameters. These can be set in the master launch file
        #self.idle_param = rospy.get_param('/idle', True)
        #rospy.loginfo('%s: GLOBAL PARAM: idle = %s', self.module_name, self.idle_param)



        # PUBLISHERS
        self.pub_face_tracking_enabled = rospy.Publisher('/head/face_tracking_enabled', Bool, queue_size=4) 
        self.pub_random_move_enabled = rospy.Publisher('/head/random_move_enabled', Bool, queue_size=4) 

        #SUBSCRIBERS

        
        #====================================================================
        # Behavior Settings

        if False: # Pretty sure none of this is used.
            # Load this behavior's parameters to the ROS parameter server
            # TODO - SEE IF THIS WORKS with new behavior module names!
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path(self.module_name.strip("/")) # remove leading slash
            param_file_path = pkg_path + '/param/param.yaml'
            rospy.loginfo('%s: Loading Params from %s', self.module_name, param_file_path)
            paramlist = rosparam.load_file(param_file_path, default_namespace=self.module_name)
            for params, ns in paramlist:
                rosparam.upload_params(ns,params)

        # Get this behavior's *GLOBAL* parameters
        # These can be set in the master launch file
        
        self.enable_people_tracking = rospy.get_param('/Idle_behavior/enable_people_tracking', True)
        #self.enable_people_tracking = rospy.get_param('~enable_people_tracking', True)
        rospy.loginfo('%s: PARAM: enable_people_tracking = %s', self.module_name, 
            self.enable_people_tracking)

        self.enable_random_movement = rospy.get_param('/Idle_behavior/enable_random_movement', True)
        rospy.loginfo('%s: PARAM: enable_random_movement = %s', self.module_name,
            self.enable_random_movement)
            

        if DBG_DISABLE_RANDOM_MOVEMENT: # OVERRIDE! Use with caution!
            self.enable_random_movement = False
        # self.enable_people_tracking = False ### TODO TODO TODO REMOVE THIS DEBUG


        rospy.loginfo("%s: init complete." % (self.module_name))


    def cleanup(self):
        rospy.loginfo('%s: Behavior complete' % self.module_name)
        # Turn off face tracking and random movement
        self.pub_face_tracking_enabled.publish(False)
        self.pub_random_move_enabled.publish(False)

        

    #====================================================================
    def execute_behavior(self, param1, param2):
        rospy.loginfo('%s: Executing behavior' % (self.module_name))
        rospy.loginfo( "Param1: '%s'", param1)
        rospy.loginfo( "Param2: '%s'", param2)


        # Configure Face Tracker Module        
        if self.enable_random_movement:
            rospy.loginfo('%s: Enabling random head movements' % (self.module_name))
            #rospy.loginfo('%s: (Make sure eb_face_tracker is running)' % (self.module_name))
            self.pub_random_move_enabled.publish(True)
        else:
            rospy.loginfo('%s: Random head movements DISABLED' % (self.module_name))
            self.pub_random_move_enabled.publish(False)


        if self.enable_people_tracking:
            # This will position the head, then begin tracking
            rospy.loginfo('%s: Enabling face tracking' % (self.module_name))
            #rospy.loginfo('%s: (Make sure eb_face_tracker is running)' % (self.module_name))
            self.pub_face_tracking_enabled.publish(True)
        else:
            rospy.loginfo('%s: Face tracking DISABLED' % (self.module_name))
            self.pub_face_tracking_enabled.publish(False)


        print( 'DBG: Idle Behavior Loop starting')


        #====================================================================
        # Loop until interrupted by another behavior (or system shutdown)
        while not rospy.is_shutdown():

            # print( 'Idle Behavior Loop')
            

            if self.interrupt_check():
                rospy.loginfo('%s: Interrupt Detected. Exiting Behavior.' % self.module_name)
                break

            rospy.sleep(0.25) # seconds

        self.cleanup()         


        
if __name__ == '__main__':
    rospy.init_node('idle_behavior')
    server = IdleBehavior()
    rospy.spin()
