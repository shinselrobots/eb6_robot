#! /usr/bin/env python

import rospy
import time
import random
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Empty

# for servos
#from eb_servos.head_servo_publishers import *
#from eb_servos.right_arm_servo_publishers import *
#from eb_servos.left_arm_servo_publishers import *

from eb_servos.set_pose import *
#from eb_servos.standard_servo_positions import *
#from eb_servos.set_servo_speed import *
#from eb_servos.set_servo_torque import *

class DangerBehavior():
    # run in circles, scream and shout!
    
    def __init__(self, behavior_utils, interrupt_check):
        self.module_name = 'danger_behavior'
        rospy.loginfo('%s: Initializing...' % (self.module_name))
        self.interrupt_check = interrupt_check
        self.send_status_update = behavior_utils.send_status_update
        self.speak = behavior_utils.speak
        self.sound_bites_dir = behavior_utils.sound_bites_dir

        
        
        self.speech_done = True
        self.imu_yaw = 0.0
        self.rotation_start = 0.0

        imu_sub = rospy.Subscriber("/imu_orientation", Point32, self.imu_callback)
        
        rospy.loginfo("%s: init complete." % (self.module_name))


    def imu_callback(self, data):
        #print("got imu: ", data) 
        #self.imu_roll = data.x + self.imu_roll_hardware_offset_degrees + (self.slider07/10.0)
        #self.imu_pitch = data.y + self.imu_pitch_hardware_offset_degrees + (self.slider11/100.0)
        self.imu_yaw = data.z


        # Sound effects
        # OLD:  "/home/system/catkin_robot/src/eb/eb_behaviors/resources/sounds/sound_bites")
        self.sound_effect_danger = os.path.join(self.sound_bites_dir, "danger_will_robinson.wav")
        rospy.loginfo("DBG: self.sound_effect_danger: %s", self.sound_effect_danger)

        #self.mic_system_enable_pub = rospy.Publisher('/microphone/system_enable', Bool, queue_size=1)

    def check_turn_complete(self):
        rotation amount = self.imu_yaw - self.rotation_start
        
    
        rospy.loginfo('%s: Rotation Completed.' % self.module_name)


    def stop_turn(self):
        # stop any wheel motion
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        self.pub_wheel_motors.publish(twist)
        print("TWIST STOP!")

    def cleanup(self):
        # un-mute the microphone
        # self.mic_system_enable_pub.publish(True)
        rospy.loginfo('%s: Behavior complete' % self.module_name)


    # -------------------------------------------------------------------------
    def execute_behavior(self, param1, param2):
        rospy.loginfo('%s: Executing behavior' % (self.module_name))
        rospy.loginfo( "Param1: '%s'", param1)
        rospy.loginfo( "Param2: '%s'", param2)


        # Publish wheel motor commands to motor control
        # TODO: Put this in launch file: 
        #   <!-- <remap from="cmd_vel" to="move_base/priority1"/>  --> 
        #self.pub_wheel_motors = rospy.Publisher('move_base/priority1', Twist, queue_size=5)
        self.pub_wheel_motors = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # mute the microphone, so the robot does not hear sounds and servos!
        #self.mic_system_enable_pub.publish(False)

        # Get rotation start position
        self.rotation_start = self.imu_yaw
        if self.rotation_start == 0.0:
            rospy.logwarn('%s: No IMU data received! May not stop at correct rotation!' % (self.module_name))
            

        
        # Publish motor command
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0.3 # SPEED OF TURN   1.0
        self.pub_wheel_motors.publish(twist)

        # play wave file "Danger, Will Robinson!" (blocking)
        # TODO DISABLE FOR TESTING!!! 
        playsound(self.sound_effect_danger)
        print("************************* PLAYING WILL ROBINSON! TODO *******************")

        # start saying phrase
        print("************************* Yelling! *******************")
        self.speak('When in danger when in doubt run in circles scream and shout, when in danger when in doubt run in circles scream and shout.', False) # Don't wait for speaking to complete


        # EB TODO - blink ears red and white
        # EB TODO move legs up and down?

        for i in range(0, 20):
            if self.interrupt_check():
                rospy.loginfo('%s: Interrupt Detected. Exiting Behavior.' % self.module_name)
                break
            self.check_turn_complete()
            
            rospy.sleep(0.25)

        # EB TODO setpose to default?   all_home() # arms/head in home position
        self.stop_turn()

        if not self.InterruptRequested():
            self.speak("Woah, what just happened? I think I blew a circuit")

        # Finish Behavior

        
        rospy.loginfo('%s: Spinning until some other behavior takes over...' % (self.module_name))
        while not rospy.is_shutdown():
            if self.interrupt_check():
                rospy.loginfo('%s: Interrupt Detected. Exiting Behavior.' % self.module_name)
                break
            rospy.sleep(0.25) # seconds

        self.cleanup()         

        
if __name__ == '__main__':
    rospy.init_node('danger_behavior')
    server = DangerBehavior(None)
    rospy.spin()
