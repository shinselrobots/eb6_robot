#! /usr/bin/env python

import rospy
import time
import random
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Empty


class NothingBehavior():
    # This behavior does nothing. Used instead of Idle behavior to avoid movement.
    def __init__(self, behavior_utils, interrupt_check):
        self.module_name = 'nothing_behavior'
        rospy.loginfo('%s: Initializing...' % (self.module_name))
        self.interrupt_check = interrupt_check
        self.send_status_update = behavior_utils.send_status_update
        self.speak = behavior_utils.speak
        
        rospy.loginfo("%s: init complete." % (self.module_name))

    def cleanup(self):
        rospy.loginfo('%s: Behavior complete' % self.module_name)

    def execute_behavior(self, param1, param2):
        rospy.loginfo('%s: Executing behavior' % (self.module_name))
        rospy.loginfo( "Param1: '%s'", param1)
        rospy.loginfo( "Param2: '%s'", param2)

        rospy.loginfo('%s: Spinning until some other behavior takes over...' % (self.module_name))
        while not rospy.is_shutdown():
            if self.interrupt_check():
                rospy.loginfo('%s: Interrupt Detected. Exiting Behavior.' % self.module_name)
                break
            #print("DBG: NOTHING behavior loop...")
            rospy.sleep(0.25) # seconds

        self.cleanup()         

        
if __name__ == '__main__':
    rospy.init_node('nothing_behavior')
    server = NothingBehavior(None)
    rospy.spin()
