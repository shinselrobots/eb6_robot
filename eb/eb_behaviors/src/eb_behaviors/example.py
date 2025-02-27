#! /usr/bin/env python

import rospy
import time
import random
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Empty
from angles import fabs, shortest_angular_distance

# for servos
#from eb_servos.head_servo_publishers import *
#from eb_servos.right_arm_servo_publishers import *
#from eb_servos.left_arm_servo_publishers import *

from eb_servos.set_pose import *
#from eb_servos.standard_servo_positions import *
#from eb_servos.set_servo_speed import *
#from eb_servos.set_servo_torque import *

class ExampleBehavior():

    def __init__(self, behavior_utils, interrupt_check):
        self.module_name = 'example_behavior'
        rospy.loginfo('%s: Initializing...' % (self.module_name))

        self.interrupt_check = interrupt_check
        self.send_status_update = behavior_utils.send_status_update
        self.speak = behavior_utils.speak
        self.play_sound = behavior_utils.play_sound
        self.music_dir = behavior_utils.music_dir
        self.sound_bites_dir = behavior_utils.sound_bites_dir
        self.robotpose = RobotPose(self.module_name)


        # Get this behavior's *GLOBAL* parameters. These can be set in the master launch file
        #self.example_param = rospy.get_param('/example', True)
        #rospy.loginfo('%s: GLOBAL PARAM: example = %s', self.module_name, self.example_param)


        foo = shortest_angular_distance(32.0, 66.0)
        print(foo)
        
        rospy.loginfo("%s: init complete." % (self.module_name))

          
    def some_subroutine(self):
        print("some_subroutine called")

    def cleanup(self):
        rospy.loginfo('%s: Behavior complete' % self.module_name)

    def execute_behavior(self, param1, param2):
        rospy.loginfo('%s: Executing behavior' % (self.module_name))
        rospy.loginfo( "Param1: '%s'", param1)
        rospy.loginfo( "Param2: '%s'", param2)

        # publishers
        # self.pub_some_publisher =   rospy.Publisher('/example',   UInt16, queue_size=10)        

        #self.speak("this is an example")

        # Example Pose move
        #lowest_servo_speed = 0.3
        #goal_pose = 3 
        #self.robotpose.move(goal_pose, lowest_servo_speed)
        
        rospy.loginfo('%s: Doing Fake Stuff...' % (self.module_name))
        time.sleep(1.0) # seconds


        # TEST INTERRUPT
        i = 10
        print("Doing work while checking for interrupt")
        while i > 0:
            if self.interrupt_check():
                print("****** EXAMPLE BEHAVIOR: interrupt detected! Ending Behavior!")
                break
            print("Doing work while waiting for interrupt")
            rospy.sleep(1.0)

        self.cleanup()         

        
if __name__ == '__main__':
    rospy.init_node('example_behavior')
    server = ExampleBehavior()
    rospy.spin()
