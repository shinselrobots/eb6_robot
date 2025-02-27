#! /usr/bin/env python

import rospy
import random
from random import randint

import math
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Empty
from std_msgs.msg import Bool


#import os, threading # for music

# for servos
#from eb_servos.head_servo_publishers import *
#from eb_servos.right_arm_servo_publishers import *
#from eb_servos.left_arm_servo_publishers import *

from eb_servos.set_pose import *
#from eb_servos.standard_servo_positions import *
#from eb_servos.set_servo_speed import *
#from eb_servos.set_servo_torque import *

#from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Point32, Pose, Quaternion, Twist, Vector3

#from eb_behaviors.move_utils import MoveUtils


# ========================================================================        
class JokeBehavior():

    def __init__(self, behavior_utils, interrupt_check):
        self.module_name = 'joke_behavior'
        rospy.loginfo('%s: Initializing...' % (self.module_name))

        self.interrupt_check = interrupt_check
        self.send_status_update = behavior_utils.send_status_update
        self.speak = behavior_utils.speak
        self.play_sound = behavior_utils.play_sound
        self.music_dir = behavior_utils.music_dir
        self.sound_bites_dir = behavior_utils.sound_bites_dir
        self.robotpose = RobotPose(self.module_name)
        self.happy_move = HappyMove(self.interrupt_check)

        # Behavior sounds are stored in the shared behavior resource directory
        self.resource_dir = rospy.get_param('resource_dir', 
          '/home/system/catkin_robot/src/eb/eb_behaviors/resources')
        #self.scripts_dir = os.path.join(self.resource_dir, 'csv_scripts')



        # constants
        self.MAX_PAN = 1.047       #  60 degrees
        self.DEGREES_90 = 1.50     #  Radians
        self.MAX_SIDETILT = 0.40   # max before hitting neck
        self.MAX_TILT_UP =  0.70   # 0.65 # 0.52    #  Limit vertical to assure good tracking
        self.MAX_TILT_DOWN = 1.10  #  max down before hitting neck
        self.TILT_CENTER = 0.45    # (IDLE behavior = 0.25)
        
        #self.CONVERSATION_TILT_CENTER = 0.25 # 0.15    #  lean toward up  (since robot is on the ground)
        self.MAX_ANTENNA_BACK = -0.7807      
        self.MAX_ANTENNA_FORWARD = 1.5
        self.JOKE_ANTENNA_CENTER = -0.3
        self.EAR_CMD_AI_MODE       = 2 # Chasing (thinking)
        self.EAR_CMD_KEYWORD_MODE  = 3 # Rainbow


        # Publishers
        self.pub_wheel_motors = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_neck_raise = rospy.Publisher('/neck_raise_joint/command', Float64, queue_size=1)
        self.pub_head_pan = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
        self.pub_head_tilt = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)
        self.pub_head_sidetilt = rospy.Publisher('/head_sidetilt_joint/command', Float64, queue_size=1)
        self.pub_right_antenna = rospy.Publisher('/right_antenna_joint/command', Float64, queue_size=1)
        self.pub_left_antenna = rospy.Publisher('/left_antenna_joint/command', Float64, queue_size=1)
        
        rospy.loginfo("%s: init complete." % (self.module_name))

      
    def publish_wheel_move_command(self, speed):
        print("DBG:publish_wheel_move_command: Speed = ", speed)
        twist = Twist()
        twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        #print("DBG: sending TWIST command: ", twist)
        self.pub_wheel_motors.publish(twist)

    def publish_wheel_turn_command(self, speed):
        print("DBG:publish_wheel_turn_command: Speed = ", speed)
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = speed
        #print("DBG: sending TWIST command: ", twist)
        self.pub_wheel_motors.publish(twist)


    # TODO - Copied from IDLE.PY  Need to make a common library and share this code
    # Track current servo positions
    def call_return_joint_states(self, joint_names):
        rospy.wait_for_service("return_joint_states")
        try:
            s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
            resp = s(joint_names)
        except rospy.ServiceException as e:
            rospy.logwarn("%s: Error when calling return_joint_states: %s" % (self.module_name, e))
        for (ind, joint_name) in enumerate(joint_names):
            if(not resp.found[ind]):
                rospy.logwarn("%s: joint %s not found!" % (self.module_name, joint_name ))
        return (resp.position, resp.velocity, resp.effort)

    def get_head_servo_positions(self):
        # returns positions of servos. Tilt normalized with neck position (so 0 = facing straight)
        (position, velocity, effort) = self.call_return_joint_states( \
            ['head_sidetilt_joint', 'head_pan_joint', 'head_tilt_joint', 'neck_raise_joint'])

        sidetilt = position[0]
        pan = position[1]
        tilt = position[2]
        neck = position[3]

        #print( " DBG:Idle Behavior: *** Current Raw Positions: sidetilt = %2.3f, pan = %2.3f, tilt = %2.3f, neck = %2.3f" % 
        #    (sidetilt, pan, tilt, neck))

        # factor in neck positon for tilt
        adjusted_tilt = tilt - neck
        #print( " DBG:Idle Behavior: *** get_head_servo_positions raw Tilt = %2.3f, Adjusted Tilt = %2.3f" % (tilt, adjusted_tilt))

        return(sidetilt, pan, adjusted_tilt, neck)


    def antenna_move(self, command, antenna = "both"):
        
        # print( "DBG:Idle Behavior: move antenna called: command = %2.3f" %  (command))
        #return
        if command > self.MAX_ANTENNA_FORWARD:
            # print( "DBG:Idle Behavior: limiting pan to MAX. Requested = %2.3f" %  (command))
            command = self.MAX_ANTENNA_FORWARD
        elif command < self.MAX_ANTENNA_BACK:
            # print( "DBG:Idle Behavior: limiting pan to -MAX. Requested = %2.3f" %  (command))
            command = self.MAX_ANTENNA_BACK

        if antenna == "both" or antenna == "right":
            self.pub_right_antenna.publish(command)
        if antenna == "both" or antenna == "left":
            self.pub_left_antenna.publish(command)


    def head_pan_move(self, command):
        
        #print( "DBG:Idle Behavior: head_pan called: command = %2.3f" %  (command))
        #return
        if command > self.MAX_PAN:
            # print( "DBG:Idle Behavior: limiting pan to MAX. Requested = %2.3f" %  (command))
            command = self.MAX_PAN
        elif command < -self.MAX_PAN:
            # print( "DBG:Idle Behavior: limiting pan to -MAX. Requested = %2.3f" %  (command))
            command = -self.MAX_PAN
        self.pub_head_pan.publish(command)
    
    def head_sidetilt_move(self, command):
        # print( "DBG:Idle Behavior: head_sidetilt called: command = %2.3f" %  (command))
        #return
        if command > self.MAX_SIDETILT:
            # print( "DBG:Idle Behavior: limiting sidetilt to MAX. Requested = %2.3f" %  (command))
            command = self.MAX_SIDETILT
        elif command < -self.MAX_SIDETILT:
            # print( "DBG:Idle Behavior: limiting sidetilt to -MAX. Requested = %2.3f" %  (command))
            command = -self.MAX_SIDETILT
        self.pub_head_sidetilt.publish(command)

    def head_tilt_move(self, command):
        #print( "DBG:Idle Behavior: head_tilt_move called: command = %2.3f" %  (command))
        #return
        if command > self.MAX_TILT_UP:
            # print( "DBG:Idle Behavior: limiting tilt to MAX_TILT_UP. Requested = %2.3f" %  (command))
            command = self.MAX_TILT_UP
       
        # normalized to neck position        
        (current_sidetilt, current_pan, current_tilt, current_neck) = self.get_head_servo_positions()
        new_tilt = current_neck + command
        # ---------------------------  
        
        #print " DBG:Idle Behavior: head_tilt_move called: command = %2.3f, neck = %2.3f, new_tilt = %2.3f" % (command, current_neck, new_tilt)

        # apply limits
        if command < -self.MAX_TILT_DOWN:
            # print( "DBG:Idle Behavior: limiting tilt to MIN. Requested = %2.3f" %  (command))
            command = self.MAX_TILT_DOWN
        
        if new_tilt < -self.MAX_TILT_DOWN:
            # print( "DBG:Idle Behavior: limiting tilt to MIN. Requested = %2.3f" %  (new_tilt))
            new_tilt = -self.MAX_TILT_DOWN
       
        self.pub_head_tilt.publish(new_tilt)
        



    def sleep_check(self, sleep_time):
        if self.interrupt_check():
            print("")
            print("**************************************************")
            print("****** JOKE: interrupt detected! Ending Behavior!")
            print("**************************************************")
            print("")
            self.cleanup()
            return True
            
        # randomly move the antennas
        leftAntennaAmt = random.uniform(-0.5, 0.5)
        self.antenna_move(self.JOKE_ANTENNA_CENTER+leftAntennaAmt, "left")

        rightAntennaAmt = random.uniform(-0.5, 0.5)
        self.antenna_move(self.JOKE_ANTENNA_CENTER+rightAntennaAmt, "right")

        # Side-tilt head a little every so often                  
        side_tilt_amt = random.uniform(-0.2, 0.2)
        self.head_sidetilt_move(side_tilt_amt)

        tiltAmt = self.TILT_CENTER + random.uniform(-0.1, 0.1)  # Normal: +/- 0.3
        self.head_tilt_move(tiltAmt)

        panAmt = random.uniform(-0.1, 0.1)
        self.head_pan_move(panAmt)
            
        rospy.sleep(sleep_time)


    def cleanup(self):
        self.publish_wheel_move_command(0.0) # Wheels - stop
        self.robotpose.move(3, 0.3) # Pose, Speed - back to normal

        rospy.loginfo('%s: Behavior complete' % self.module_name)


    # --------------------------------------------------------------------------------------------
    def execute_behavior(self, param1, param2):
        rospy.loginfo('%s: Executing behavior' % (self.module_name))
        rospy.loginfo( "Param1: '%s'", param1)
        rospy.loginfo( "Param2: '%s'", param2)

        normal_servo_speed = 0.5
        eye_color_normal = 0x00002f # default blue
        eye_color_bright = 0x0000ff # bright blue 
        punchline_time = 1.5
        interval_time = 3.0


        # TODO Save / Restore servo speeds
        # Set servos speed and torque
        #SetServoTorque(1.0, head_joints)
        #SetServoSpeed(normal_servo_speed, neck_joints)  # 0.35,0 .5
        #SetServoSpeed(1.0, antenna_joints)  # 0.35

        # Center Camera Head
        ##self.head_pan_move(0.0)
        ##self.head_tilt_move(self.JOKE_TILT_CENTER) 
        #self.head_sidetilt_move(0.0)
        #self.antenna_move(self.JOKE_ANTENNA_CENTER, "both")
        #panAmt = random.uniform(-0.5, 0.5)
        #self.head_pan_move(panAmt)
        # examples
        #self.publish_wheel_turn_command(0.3) # Wheels - turn LEFT
        #self.publish_wheel_turn_command(0.0) # Wheels - Stop


        self.speak("Hey, would you like to hear some jokes?")

        #if self.sleep_check(2.0):
        #    return

        self.speak("okay, R2 D2, told me some star wars jokes") 
        #self.happy_move.move(5) # Number of leans. blocks until done
        ##self.head_tilt_move(self.JOKE_TILT_CENTER + 0.10)
        if self.sleep_check(1.0):
            return

        self.speak("Lets see if you like them") 
        #self.happy_move.move(5) # Number of leans. blocks until done

        ##self.head_pan_move(0.5)
        if self.sleep_check(2.0):
            return
        self.speak("What do you call Chew bock ah when he has chocolate in his hair?")
        ##self.head_pan_move(0.5)
        if self.sleep_check(punchline_time):
            return
        self.speak("a chocolate chip, wookie")

        ##self.head_pan_move(-0.5)
        ##self.head_tilt_move(self.JOKE_TILT_CENTER)
        if self.sleep_check(interval_time):
            return
        self.speak("how does darth vader like his steak cooked?") # setup

        ##self.head_pan_move(0.0)
        ##self.head_tilt_move(self.JOKE_TILT_CENTER + 0.05)
        if self.sleep_check(punchline_time):
            return
        self.speak("a little on the dark side") # punchline

        ##self.head_pan_move(0.3)
        if self.sleep_check(interval_time):
            return
        self.speak("Do you know why see three pee oh is embarrassed?") # setup

        ##self.head_pan_move(0.3)
        ##self.head_tilt_move(self.JOKE_TILT_CENTER)
        if self.sleep_check(punchline_time):
            return
        self.speak("Because he has hard ware and he has soft ware") 
        ##self.head_tilt_move(self.JOKE_TILT_CENTER - 0.02)
        if self.sleep_check(0.5):
            return
        self.speak("but he does not have under ware") # punchline

        ##self.head_pan_move(-0.5)
        ##self.head_tilt_move(self.JOKE_TILT_CENTER)
        if self.sleep_check(interval_time):
            return


        self.speak("what do you say when Luke Sky walker is eating with his hands?") # setup

        ##self.head_pan_move(0.5)
        ##self.head_tilt_move(self.JOKE_TILT_CENTER + 0.02)
        if self.sleep_check(punchline_time):
            return
        self.speak("Use the fork, Luke") # punchline

        self.head_pan_move(0.0)
        ##self.head_tilt_move(self.JOKE_TILT_CENTER)
        if self.sleep_check(interval_time):
            return

        self.speak("Well, I hope you liked my jokes") # setup
        if self.sleep_check(1.0):
            return


        self.robotpose.move(3, 0.3) # Pose, Speed - back to normal
         
        self.cleanup()         

# ========================================================================        
class HappyMove():
    # make happy movements, such as leaning back and forth

    def __init__(self, interrupt_check):
        self.module_name = 'happy_move'
        rospy.loginfo('%s: Initializing...' % (self.module_name))
        self.interrupt_check = interrupt_check
        self.robotpose = RobotPose(self.module_name)
        self.start_thigh_lift = 0.0
        self.start_knee_bend = 0.0
        self.start_ankle_rotate = 0.0
 
        rospy.loginfo("%s: init complete." % (self.module_name))
            

    def move(self, number_of_leans=3):
        sleep_time = 0.25  # seconds
        lean_lowest_servo_speed = 0.3

        self.starting_pose = self.robotpose.get_current_pose() # save pose to restore it later
        rospy.loginfo('%s: Saving Current pose: [%d]' % (self.module_name, self.starting_pose))
        if self.starting_pose < 2:
            # Happy movements requre a standing pose
            pose_lowest_servo_speed = 0.3 
            goal_pose = 3
            self.starting_pose = goal_pose
            self.robotpose.move(goal_pose, pose_lowest_servo_speed)

        # Possible Optimization: Set servo speeds here instead of in the loop
        # (for example: self.robotpose.set_lean_speeds(lowest_servo_speed))

        for i in range(0, number_of_leans):
            if self.interrupt_check():
                rospy.loginfo('%s: Interrupt Detected. Exiting Behavior.' % self.module_name)
                break

            #print("DBG: sleep_time = %0.2f", sleep_time)
            lean_lowest_servo_speed = 0.3  # speed range from 0.3 to 2.3 
            #print("DBG: lean servo_speed = %0.2f", lean_lowest_servo_speed)

            direction = 'RIGHT'
            success = self.robotpose.lean(direction, lean_lowest_servo_speed)
            if not success:
                rospy.logwarn('%s: Robotpose.lean error. Exiting Behavior.' % self.module_name)
                break

            rospy.sleep(sleep_time)

            direction = 'LEFT'
            success = self.robotpose.lean(direction, lean_lowest_servo_speed)
            if not success:
                rospy.logwarn('%s: Robotpose.lean error. Exiting Behavior.' % self.module_name)
                break
         
            rospy.sleep(sleep_time)

        direction = 'CENTER'
        success = self.robotpose.lean(direction, lean_lowest_servo_speed)
        if not success:
            rospy.logwarn('%s: Robotpose.lean error.' % self.module_name)
 
        # Done - Set Pose back to initial pose?
        # lowest_servo_speed = 0.3
        #goal_pose = self.starting_pose
        #self.robotpose.move(goal_pose, lowest_servo_speed)

        rospy.loginfo('Happy complete')



        
if __name__ == '__main__':
    rospy.init_node('joke_behavior')
    server = JokeBehavior()
    rospy.spin()
