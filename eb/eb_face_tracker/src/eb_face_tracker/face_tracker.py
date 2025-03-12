#! /usr/bin/env python

import rospy
import time
import random
from random import randint
import math
from math import radians, degrees
import os 
import _thread

import rospkg
import rosparam
from sensor_msgs.msg import LaserScan

from std_msgs.msg import Bool
from std_msgs.msg import UInt16
from std_msgs.msg import UInt32
from std_msgs.msg import Float64
from std_msgs.msg import String
from system_status_msgs.msg import SystemStatus

# import tf # Warning - needs recompile for Python3. See: 
#  https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/

# for servos
from eb_servos.set_pose import *
from eb_servos.servo_joint_list import head_joints
from eb_servos.head_servo_publishers import *
from eb_servos.set_servo_speed import *
from eb_servos.set_servo_torque import *

from eb_servos.head_control import HeadControl

from eb_servos.srv import ReturnJointStates

from body_tracker_msgs.msg import BodyTracker, BodyTrackerArray
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose, Pose2D
#import geometry_msgs.msg

SERVO_SPEED = 0.35

class BoundingBox():
    def __init__(self, left, top, width, height):
        self.bb_left =   left
        self.bb_top =    top
        self.bb_width =  width
        self.bb_height = height


class FaceTracker():

    def __init__(self):
        rospy.init_node('face_tracker')
        self.log_name = 'face_tracker'
        rospy.loginfo('%s: Initializing...' % (self.log_name))

        self.head_control = HeadControl(self.log_name)
        
        # CONSTANTS
        self.TILT_CENTER = 0.25 # 0.15    #  lean toward up in random movements (since robot is on the ground)
        #self.ANTENNA_CENTER = -0.4
        self.MAX_ANTENNA_BACK = -0.7807
        self.MAX_ANTENNA_FORWARD = 1.5
        self.ANTENNA_LISTENING = 0.7
        self.ANTENNA_NOT_LISTENING = -0.1

        self.EAR_CMD_AI_MODE       = 2 # Chasing (thinking)
        self.EAR_CMD_KEYWORD_MODE  = 3 # Rainbow

        # listen_state For Antenna movement
        self.SPEECH_RECO_NOT_LISTENING = 0
        self.SPEECH_RECO_LISTENING = 1
        self.SPEECH_RECO_IDLE = 2
        
        self.NAME_TIMEOUT_SECS = 10.0
        self.FACE_INSIDE_BODY_BB_TIMEOUT_SEC = 2.0


        self.DEADBAND_ANGLE = 0.1 # DEBUG
        #self.DEADBAND_ANGLE = 0.0872665 # 5 deg deadband in middle to prevent osc

        #NORMAL TODO self.DEADBAND_ANGLE = 0.2 # ? deg deadband in middle to prevent osc


        # WIDE ANGLE LENSE: self.DEADBAND_ANGLE = 0.32 # 0.0872665 # 5 deg deadband in middle to prevent osc
        ##self.ai_mode_active = False
        self.tracking = False
        self.tracking_received = False
        self.enable_people_tracking = True # TODO False # start with movement off, until enabled
        self.enable_random_movement = True
        self.require_body_tracking = False
        self.body_list = []
        self.antenna_center = self.ANTENNA_NOT_LISTENING
        self.tracking = False
        #self.joint_state = JointState() # for reading servo positions

        # Remember which person we are tracking
        #self.current_person_id = 0  # 0 = not tracking anyone
        #self.current_person_id_time = rospy.Time.now() # start timer
        self.named_person = ""
        ##self.named_person_id = 0
        self.named_person_time = rospy.Time.now() # start timer
        self.face_inside_body_bb = False
        self.face_inside_body_bb_time = rospy.Time.now() # start timer
        
        self.named_people_seen_today = {}
        self.reco_listen_state = self.SPEECH_RECO_IDLE
        self.speech_reco_state = ''
        
        self.timeStamp = time.time()
        self.fpsSmooth = 0
        self.laser_data = None # for finding distance to people
        laser_scan_topic = rospy.get_param("~laser_topic", "/scan")
        rospy.loginfo("Listening on Laser Topic: " + laser_scan_topic)
        self.max_laser_max_range = rospy.get_param("~laser_max_range", 2.00)

        
        # PUBLISHERS
        self.status_pub = rospy.Publisher('/system_status', SystemStatus, queue_size=8)
        self.pub_head_pan = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
        self.pub_head_tilt = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)
        self.pub_head_sidetilt = rospy.Publisher('/head_sidetilt_joint/command', Float64, queue_size=1)
        self.pub_right_antenna = rospy.Publisher('/right_antenna_joint/command', Float64, queue_size=1)
        self.pub_left_antenna = rospy.Publisher('/left_antenna_joint/command', Float64, queue_size=1)

        # Publish tracking status and current person by name, if recognized
        self.pub_tracking_state = rospy.Publisher('/person/tracking', Bool, queue_size=2) 
        self.pub_current_user_name = rospy.Publisher('/person/name', String, queue_size=2) 


        # SUBSCRIBERS
        rospy.Subscriber("/head/face_tracking_enabled", Bool, self.face_tracking_enabled_callback)
        rospy.Subscriber("/head/random_move_enabled", Bool, self.random_move_enabled_callback)

        # For finding closest person to track:
        rospy.Subscriber(laser_scan_topic, LaserScan, self.laser_callback)

        # For moving antennas:
        self.status_update_sub = rospy.Subscriber("/system_status", SystemStatus, self.status_update_callback)

        # Messages from face recognizer to identify person in center of camera image
        face_reco_sub = rospy.Subscriber("/body_tracker_array/center_face_name", \
            String, self.face_recognition_callback, queue_size=1)
        
        # Message from face detector for face tracking
        face_sub = rospy.Subscriber("/body_tracker_array/face", \
            BodyTrackerArray, self.face_callback, queue_size=1)

        # Message from body detector for body tracking
        body_sub = rospy.Subscriber("/body_tracker_array/body", \
            BodyTrackerArray, self.body_callback, queue_size=1)


        # Message Buffer example (if needed in future):
        #face_sub = rospy.Subscriber("/body_tracker_array/face", \
        #    BodyTrackerArray, self.face_callback, queue_size = 1, buff_size = (256*10) ) 
        # room for about 10 messages, so they don't back up

 
        if self.enable_people_tracking or self.enable_random_movement:
            # if enabled while starting set servo speeds
            self.set_head_servo_speeds()


        self.send_status_update("FACE_TRACKER", "WAIT_FOR_FACE")

        rospy.loginfo("%s: init complete." % (self.log_name))
        #====================================================================

    ##def ai_mode_enabled_callback(self, data):
        ##self.ai_mode_active = data.data
 
    def listen_state_callback(self, data):
        self.reco_listen_state = data.data
        # SPEECH_RECO_NOT_LISTENING = 0
        # SPEECH_RECO_LISTENING = 1
        # SPEECH_RECO_IDLE = 2

        # Indicate listen state by moving antennas
        if self.reco_listen_state == self.SPEECH_RECO_NOT_LISTENING:
            self.head_control.antenna_move(self.ANTENNA_NOT_LISTENING, "left")
            self.head_control.antenna_move(self.ANTENNA_NOT_LISTENING, "right")

        elif self.reco_listen_state == self.SPEECH_RECO_LISTENING:
            self.head_control.antenna_move(self.ANTENNA_LISTENING, "left")
            self.head_control.antenna_move(self.ANTENNA_LISTENING, "right")

        # ignore SPEECH_RECO_IDLE?
 
 
    def status_update_callback(self, status_msg):
        # Gets updates from modules and indicates listen status with antennas
        # Located in this module, since it controls antenna position.
        # Note that this module publishes to and subscribes to topic "/system_status")

        item_str = status_msg.item
        status_str = status_msg.status
        #rospy.loginfo("%s: Got Status update [%s] [%s]" % (self.log_name, item_str, status_str))            
        

        if item_str == 'SPEECH_RECO_STATE': # speech recognition server state
            self.speech_reco_state = status_str 

            # Speech Recogniton State
            if self.speech_reco_state == "LISTENING":
                self.antenna_center = self.ANTENNA_LISTENING
                self.head_control.antenna_move(self.antenna_center, "both")

            elif self.speech_reco_state == "LISTEN_DONE":
                self.antenna_center = self.ANTENNA_NOT_LISTENING
                self.head_control.antenna_move(self.antenna_center, "both")

            # other states don't effect the antennas, so they are ignored 
 
 
    def face_recognition_callback(self, msg):
        # Message with name of person recognized
        # Name stays until timed out or tracking disabled
        if msg.data != '':
            rospy.loginfo("%s: Got User Name: [%s]" % (self.log_name, msg.data))
            self.named_person = msg.data
            self.named_person_time = rospy.Time.now()
            self.pub_current_user_name.publish(self.named_person)
            self.send_status_update("FACE_DETECTOR", "GOT_NAME")
        

    def face_tracking_enabled_callback(self, data):
        self.enable_people_tracking = data.data
        if self.enable_people_tracking:
            rospy.loginfo("%s: Face Tracking ENABLED" % (self.log_name))
            self.set_head_servo_speeds()
            # Position head camera for face tracking
            
            self.head_control.head_pan_move(0.0)
            # tilt head up to find people more easily
            self.head_control.head_tilt_move(self.TILT_CENTER) 
            self.head_control.head_sidetilt_move(0.0)
            self.head_control.antenna_move(self.antenna_center, "both")
            
        else:
            rospy.loginfo("%s: Face Tracking DISABLED" % (self.log_name))
            self.named_person = ''
            self.pub_current_user_name.publish(self.named_person)

    
    def random_move_enabled_callback(self, data):   
        self.enable_random_movement = data.data
        if self.enable_random_movement:
            rospy.loginfo("%s: Random Movement ENABLED" % (self.log_name))
            self.set_head_servo_speeds()
        else:
            rospy.loginfo("%s: Random Movement DISABLED" % (self.log_name))


    def send_status_update(self, item, status):
        # Send status update to system monitor
        status_msg = SystemStatus()
        status_msg.item = item
        status_msg.status = status
        self.status_pub.publish(status_msg)


    def set_head_servo_speeds(self):
        # TODO Save / Restore servo speeds?
        # Set servos speed and torque
        # SetServoTorque(1.0, head_joints)
        if self.enable_people_tracking or self.enable_random_movement:
            SetServoSpeed(SERVO_SPEED, neck_joints)  # 0.35
            SetServoSpeed(1.0, antenna_joints)  # 0.35

    def laser_callback(self, data):
        self.laser_data = data

    def find_laser_distance(self, camera_angle):
        # get distance to person at supplied angle from laser (depth camera)
        # multiple samples taken and closest valid sample used for range            

        if self.laser_data is None:
            rospy.logwarn("%s: No Laser data yet..." % (self.log_name))
            return self.max_laser_max_range
            

        angle_increment = self.laser_data.angle_increment
        laser_angle_min = self.laser_data.angle_min # negative radians from center
        
        number_of_laser_values = len(self.laser_data.ranges)
        sample_width = radians(5.0) # number of degrees to sample for person range
        number_of_samples_to_measure = int(radians(5.0) / angle_increment)

        range_value =  self.max_laser_max_range       
        target_index = int((camera_angle - laser_angle_min) / angle_increment)
        start_index = target_index - int(number_of_samples_to_measure / 2)
        end_index = start_index + number_of_samples_to_measure

        # center_value = self.laser_data.ranges[target_index]
        #print("DBG: target_index = %d" % (target_index)) # 0-848
        #print("DBG: center value = %2.4f" % (center_value)) # meters

        if start_index < 0:
            start_index = 0
        if end_index > len(self.laser_data.ranges) - 1:
            end_index = len(self.laser_data.ranges) - 1

        for i in range(start_index, end_index):
                   
            r = self.laser_data.ranges[i] 

            if math.isinf(r) == True or math.isnan(r) == True:
                r = self.max_laser_max_range

            if r < range_value:
                range_value = r 
                #print("DBG: Sample [%d] = [%1.4f]" % (i, r))       
           
        #print("DBG: Total samples = %d, angle inc = %1.6f" % 
        #    (len(self.laser_data.ranges), angle_increment)) # 848

        return(range_value)
    


    #====================================================================
    # 2D Body Tracking:  Message contains body bounding box and other info
    #                    position is relative to the robot head camera.  
    def body_callback(self, msg):
    
        if not self.require_body_tracking:
            return    
    
        # print("------------------------------")
        #rospy.loginfo('%s: got body_callback message' % (self.log_name))
        #print(msg)

        self.body_list = []
        
        for i, person in enumerate(msg.detected_list):
            # print("DBG:face_tracker: enum loop = ", i)

            # create a list of bounding boxes where people are seen
            # face detector will compare to this to assure a real face
            
            body = BoundingBox(                
                person.bb_left, person.bb_top, person.bb_width, person.bb_height)

            self.body_list.append(body)               


    
    #====================================================================
    # 2D Face Tracking:  Message contains face horizontal (x) and vertical (y)
    #               position relative to the robot head camera.  
    def face_callback(self, msg):
        # Note: we don't use person name in this message, because all people seen are listed.
        # Instead, we use the center person sent in a separate message
    
        # print("------------------------------")
        #rospy.loginfo('%s: got face_callback message' % (self.log_name))
        #print(msg)

        # Calculate Time between frames
        dt=time.time()-self.timeStamp
        self.timeStamp=time.time()
        #print()
        #print("DBG:face_tracker:           Time since last face frame = %4.3f" % dt)



        #if self.servos_are_moving():
            # to prevent oscillations, wait until the servo stops to get next frame
            #rospy.loginfo('%s: SERVOS ARE MOVING. SKIPPING FRAME' % (self.log_name))
            #return
            
        # determine highest priority person to track
        # Message contains an array of people being tracked 
        # Note: tracking while stationary / idle is different from when person-following
  
        person_to_track_id = 0    
        person_to_track_index = 0    

        # Priority 1:  someone making a gesture         
        for i, person in enumerate(msg.detected_list):
            if person.gesture > -1:
                person_to_track_id = person.body_id
                person_to_track_index = i
                rospy.loginfo("DBG:face_tracker: GOT A GESTURE ") 
                break
 
        # Priority 2: Track the closest person 
        # this allows robot to change focus to different people, 
        # whoever is closest he will talk to. 
        # TODO? Track closest person that has face detected to reject tracking objects?              
        if person_to_track_id == 0:
            closest_person_distance = 100000
            closest_person_index = 0
            #print('DBG:face_tracker: getting msg detected list')
            #print(msg.detected_list)
            for i, person in enumerate(msg.detected_list):
                # print("DBG:face_tracker: enum loop = ", i)
 
                # Compare face location with body tracker to assure it's a real face
                # the face tracker sometimes triggers on random patterns, 
                # the body tracker is more robust, but updates slowly
                if self.require_body_tracking:
                    #print("DBG: Person ID = ", person.body_id)
                    #print("DBG: Person DUMP: ", person)

                    
                    for body in self.body_list:

                        body_right =  body.bb_left + body.bb_width
                        face_right =  person.bb_left + person.bb_width
                        
                        #print("DBG: Body BB Left: %d, Right: %d, Top: %d, Width: %d, Height: %d" % 
                        #    (body.bb_left, body_right, body.bb_top, body.bb_width, body.bb_height))
                        #print("DBG: Face BB Left: %d, Right: %d" % (person.bb_left, face_right))

                        if (person.bb_left > body.bb_left) and (face_right < body_right):
                            self.face_inside_body_bb = True
                            self.face_inside_body_bb_time = rospy.Time.now() # start timer
                            
                            print("*******************************************************")
                            rospy.loginfo("%s:  FOUND FACE (%d) INSIDE A BODY_BB" % 
                                (self.log_name, person.body_id))
                            print("*******************************************************")
                        #else:
                        #    print("DBG: Face not inside body bb")


                if self.face_inside_body_bb or not self.require_body_tracking:

                    # find person distance from body radar
                    laser_distance = self.find_laser_distance(person.base_to_target_polar.x)
                    print("DBG: face_tracker: Est.  Distance for person [%d] = %6.4f" % (i, person.camera_to_target_polar.z))
                    print("DBG: face_tracker: Laser Distance for person [%d] = %2.4f" % (i, laser_distance))
                    
                    if person.camera_to_target_polar.z < closest_person_distance:
                        closest_person_distance = person.camera_to_target_polar.z # TODO BUG should be Laser distance!?
                        # print("DBG:face_tracker: Tracking closest person. Distance = %2.4f" % closest_person_distance)
                        person_to_track_id = person.body_id
                        person_to_track_index = i

                elif self.require_body_tracking:
                    rospy.logwarn("%s: FACE [%d] IS NOT INSIDE BODY BB, SKIPPING FACE" % 
                        (self.log_name, person.body_id))
 


        # print("DBG:face_tracker: person_to_track_id = ", person_to_track_id)
        if person_to_track_id != 0:

            # found someone to track
            face_info = msg.detected_list[person_to_track_index]
            # if face_info.face_found == True:
                # rospy.loginfo("%s: Face Found.  Name = %s" % (self.log_name, face_info.name))

            # Track person. position in radians from center of camera
            delta_angle_x = face_info.camera_to_target_polar.x # * (-1.0) 
            delta_angle_y = face_info.camera_to_target_polar.y  
            #person_tracking_distance = msg.camera_to_target_polar.z

            # Uncomment this to debug
            # print('Delta X: %1.4f Y: %1.4f' % (delta_angle_x, delta_angle_y) )
            
            #rospy.loginfo("%s: Tracking Person Index: %d, ID: %d x: %f y: %f", \
            #    self.log_name, person_to_track_index, person_to_track_id, delta_angle_x, delta_angle_y ) 


            # Get the current servo pan and tilt position, already normalized to neck position        
            (current_sidetilt, current_pan, current_tilt, current_neck) = self.head_control.get_head_servo_positions()
            #print('DBG:face_tracker: Current Servos: Pan = %1.4f Tilt = %1.4f' % (current_pan, current_tilt) )

            # OVERRIDE
            if False:
                # Get the servo positions at the time of the frame, already normalized to neck position
                current_pan = person.servo_pan
                current_tilt = person.servo_tilt
                #print('DBG:face_tracker: Message Servos: Pan = %1.4f Tilt = %1.4f' % (current_pan, current_tilt) )
            

            if self.enable_people_tracking:
                # Tracking movement allowed

                # Actual position (from base front-center)
                base_target_x =  face_info.base_to_target_polar.x

                # add target position to current servo position
                undershoot_scale = 0.30 # percent of target distance to move
                pan_angle  = current_pan  + (delta_angle_x * undershoot_scale) #shoot for less
                tilt_angle = current_tilt + (delta_angle_y * undershoot_scale)
                #rospy.loginfo("%s: Before Clamp: Servo Command:  Pan = %f,  Tilt = %f", 
                #    self.log_name, pan_angle, tilt_angle)

                #rospy.loginfo("%s:DBG: Pan Delta: %4.2f" % (self.log_name, degrees(delta_angle_x)) ) 

                # command servos to move to target, if not in deadband
                pan_on_target = True
                tilt_on_target = True


                if abs(delta_angle_x) > self.DEADBAND_ANGLE:
                    pan_on_target = False
                    self.head_control.head_pan_move(pan_angle)    # Send servo command
                    #rospy.loginfo("%s:DBG: MOVING PAN SERVO" % self.log_name) 
                else:
                    pan_on_target = True
                    pan_angle  = current_pan         # prevent oscillations of sidetilt
                    #print('DBG:face_tracker: in X Deadband')


                if abs(delta_angle_y) > self.DEADBAND_ANGLE:
                    tilt_on_target = False
                    self.head_control.head_tilt_move(tilt_angle)  # Send servo command (auto compensated to neck position)

                else:
                    tilt_on_target = True
                    tilt_angle = current_tilt        # prevent oscillations of sidetilt
                    #print('DBG:face_tracker: in Y Deadband')

                # Pan and Tilt are offset by neck, so head is tilted.
                # Compensate for Pan and Tilt to level the sidetilt
                sidetilt_angle = pan_angle * tilt_angle * -1
                smooth_sidetilt_angle = (.5 * current_sidetilt) + (.5 * sidetilt_angle)            
                
                # print('DBG:face_tracker: SideTilt = %2.1f' % smooth_sidetilt_angle)
                self.head_control.head_sidetilt_move(smooth_sidetilt_angle)     # Send servo command

                
                #if pan_on_target and tilt_on_target:
                #    print('DBG:face_tracker: X and Y in Deadband')
                #    # rospy.loginfo("%s: On target ID %d", self.log_name, self.named_person_id)
                #else: 
                #   rospy.loginfo("%s: ID %d: Pan delta = %f, Tilt Delta = %f", 
                #     self.log_name, self.named_person_id, delta_angle_x, delta_angle_y) 

                # Max pan/tilt is constrained by system.  Add additional constraints if needed

            self.tracking = True # don't do large idle movements
            self.tracking_received = True # we received a frame from face detector
            self.send_status_update("FACE_DETECTOR", "RUNNING")


        #self.last_target_time = rospy.Time.now() # reset timer

        




    #====================================================================
    def run(self):
        rospy.loginfo('%s: Running' % (self.log_name))

        
        if self.enable_random_movement:
            rospy.loginfo('%s: random head movements enabled...' % (self.log_name))
        else:
            rospy.loginfo('%s: random head movements DISABLED' % (self.log_name))

        if self.enable_people_tracking:
            rospy.loginfo('%s: waiting for person tracking...' % (self.log_name))
        else:
            rospy.loginfo('%s: person tracking DISABLED' % (self.log_name))

        rospy.loginfo('%s: main loop starting' % (self.log_name))

        #tracking_score = 0
        #not_tracking_score = 0
        #====================================================================
        # Loop until shutdown
        while not rospy.is_shutdown():

            # print( 'Face Tracker Loop')
            #if current_neck > -0.95:
            # Only move if robot is not in sleep position? (should be overkill - handled elsewhere?)

            # Body bounding box updates slow, so we allow time for it to catch up with face position
            # see if it has timed out
            if self.face_inside_body_bb:
                body_bb_elapsed_time = rospy.Time.now() - self.face_inside_body_bb_time 
                if body_bb_elapsed_time > rospy.Duration.from_sec(self.FACE_INSIDE_BODY_BB_TIMEOUT_SEC):
                    self.face_inside_body_bb = False



            # Check to see if person's name has timed out
            time_since_last_name = rospy.Time.now() - self.named_person_time 
            if time_since_last_name > rospy.Duration.from_sec(self.NAME_TIMEOUT_SECS):
                if self.named_person != '':
                    rospy.loginfo("%s: User Name [%s] Timed out", self.log_name, self.named_person)
                self.named_person = "" 
                self.pub_current_user_name.publish(self.named_person)

            if not self.tracking_received:            
                self.send_status_update("FACE_TRACKER", "WAIT_FIRST_FACE")
                self.pub_tracking_state.publish(False)
 
            elif not self.enable_people_tracking:            
                self.send_status_update("FACE_TRACKER", "DISABLED")
                self.pub_tracking_state.publish(False)
                 
            elif self.tracking:
                self.pub_tracking_state.publish(True)
                if self.named_person != "":
                    self.send_status_update("FACE_DETECTOR", "GOT_NAME") # Tracking and have a name
                else:
                    self.send_status_update("FACE_TRACKER", "TRACKING")
                #print("DEBUG: TRACKING! ++++++++++++++++++++++++++++++++++++++++++++++++++")
                #tracking_score = tracking_score + 1

            else:
                self.pub_tracking_state.publish(False)
                self.send_status_update("FACE_TRACKER", "READY")
                #print("DEBUG: NOT TRACKING! xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
                #not_tracking_score = not_tracking_score + 1

            #print("DEBUG: SCORE: TRACKING = %4d, NOT TRACKING = %4d" % (tracking_score, not_tracking_score))
                

            if not self.tracking and self.enable_random_movement:   
                # Idle: Move head to constrained random location, at random intervals
                #rospy.loginfo('%s: Doing Random Movement' % (self.log_name))
                
                tiltAmt = self.TILT_CENTER + random.uniform(-0.3, 0.3)  # Normal: +/- 0.3
                self.head_control.head_tilt_move(tiltAmt)

                panAmt = random.uniform(-0.5, 0.5)
                self.head_control.head_pan_move(panAmt)


            self.tracking = False  # do Idle if tracking gets lost

            # delay before next loop
            randSleep = random.randint(10, 35) # tenth seconds
            for i in range(1, randSleep): 

                # do small movements between head moves
                if self.enable_random_movement:
                    if (i == 10): #  and (randSleep > 15):
                        #print("DBG:face_tracker: doing sidetilt. randSleep = ", randSleep)
                        sidetiltAmt = random.uniform(-0.2, 0.2)
                        self.head_control.head_sidetilt_move(sidetiltAmt)
                   
                    # if i == 5 + int(randSleep / 2):
                    if i % 3 == 0:


                        if self.speech_reco_state != "LISTENING":
                            # only move antennas if they aren't pointed forward in listen mode                    
                            if random.randint(0, 3) == 1:
                                # Move antenna every so often                  
                                leftAntennaAmt = random.uniform(-0.3, 0.3)
                                self.head_control.antenna_move(self.antenna_center+leftAntennaAmt, "left")

                            if random.randint(0, 3) == 1:
                                rightAntennaAmt = random.uniform(-0.3, 0.3)
                                self.head_control.antenna_move(self.antenna_center+rightAntennaAmt, "right")
            
                time.sleep(0.1)
            # KLUDGE - Set servo speeds each time we loop, as some other process may mess with servo speeds (eg. set_pose)
            self.set_head_servo_speeds()

        # End of while not rospy.is_shutdown() loop

        # cleanup        
        # leave body led in known (but not tracking) state (not sure if this ever gets executed during node shutdown
        if self.tracking_received:            
            self.send_status_update("FACE_TRACKER", "READY") 
        else:
            self.send_status_update("FACE_TRACKER", "WAIT_FIRST_FRAME")
        print("FACE TRACKER EXIT!")
        
if __name__ == '__main__':
    # capture SIGINT signal, e.g., Ctrl+C
    #signal.signal(signal.SIGINT, signal_handler)
    node = FaceTracker()
    node.run()       

