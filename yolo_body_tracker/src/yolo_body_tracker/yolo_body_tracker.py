#!/usr/bin/env python3
# ROS node for detecting and tracking people, using yolo and DeepSort
# uses sample code from: 
# https://thepythoncode.com/article/real-time-object-tracking-with-yolov8-opencv

import cv2
import sys
import numpy
import time
import csv
import os
from math import radians, degrees

#Yolo code:
import datetime
from ultralytics import YOLO
#import cv2
#from helper import create_video_writer
from deep_sort_realtime.deepsort_tracker import DeepSort



# ROS
import rospy
import logging
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from body_tracker_msgs.msg import BodyTracker
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose, Pose2D
from body_tracker_msgs.msg import BodyTrackerArray

from eb_servos.servo_joint_list import head_joints
from eb_servos.srv import ReturnJointStates
from system_status_msgs.msg import SystemStatus

#import rospkg
# Get the package directory
#rospack = rospkg.RosPack()
#cd = rospack.get_path('body_tracker')




# GLOBALS / CONSTANTS
BODY_TRACKER_SLEEP_TIME = 0.2
FRAMES_TO_SKIP = 3

CONFIDENCE_THRESHOLD = 0.8
BLUE =  (255, 0, 0)
GREEN = (0, 255, 0)
RED =   (0, 0, 255)
WHITE = (255, 255, 255)



SCALE_DISPLAY = 0.25 # scale down the display by this amount
MIRROR_DISPLAY = False
#CAMERA_NAME = '/dev/video4' # Realsense RGB
#CAMERA_NAME = '/dev/video6' # Eye USB wide angle

# For distance estimation:
AVG_FACE_WIDTH = 0.200 # width of an "average" face in meters
#USB_WIDE_CAM_1280_FOCAL_LENGTH = 533  # Measured constant EB-6 Eye at 1280 width resolution
#USB_WIDE_CAM_1920_FOCAL_LENGTH =  800 # Measured constant EB-6 Eye at 1920 width resolution
USB_WIDE_CAM_FOCAL_SCALE =      0.416  # times the width resolution

#REALSENSE_1920_RGB_FOCAL_LENGTH =  1200 # Measured constant at 1920 width resolution
#REALSENSE_848_RGB_FOCAL_LENGTH =  530 # Measured constant at 848 width resolution 
REALSENSE_CAM_FOCAL_SCALE =      0.625  # times the width resolution

#CAMERA_FOCAL_SCALE =   REALSENSE_CAM_FOCAL_SCALE   # If using Realsense Camera
CAMERA_FOCAL_SCALE =   USB_WIDE_CAM_FOCAL_SCALE     # If using EB-6 Eye Camera


# Camera Field Of View in radians
# 0.01745 radians per degree
ASTRA_MINI_FOV_X =  1.047200   # (60 degrees horizontal)
ASTRA_MINI_FOV_Y = -0.863938   # (49.5 degrees vertical)

RASPBERRY_PI_CSI_FOV_X =  2.79200   # (160 degrees horizontal)
RASPBERRY_PI_CSI_FOV_Y = -2.09749   # (120.2 degrees vertical)

#USB_WIDE_FOV_X =  2.61800   # (150 degrees horizontal)
#USB_WIDE_FOV_Y = -1.95826   # (112.2 degrees vertical)
#USB_WIDE_FOV_Y = -1.89020   # (108.3 degrees vertical)   which is correct?
#USB_WIDE_FOV_Y = -1.472625  # (84.4 degrees vertical) calculated at 16:9 ratio

USB_WIDE_FOV_X =  2.09   # (120 degrees horizontal)
USB_WIDE_FOV_Y = -1.0 * USB_WIDE_FOV_X * 9 / 16 # calculated at 16:9 ratio

# Set Camera type here
FOV_X = USB_WIDE_FOV_X
FOV_Y = USB_WIDE_FOV_Y

# Class to get current head orientaton to avoid latency issues with head-mounted cameras
# WARNING - This code is Robot hardware dependent! You must make sure it's correct for each robot
class HeadOrientation:

    def __init__(self):
        rospy.loginfo('HeadOrientation: waiting for service: return_joint_states')        
        rospy.wait_for_service("return_joint_states")
        rospy.loginfo('HeadOrientation: return_joint_states service is ready')        

    # Track current servo positions
    def call_return_joint_states(self, joint_names):
    
        #rospy.logwarn('HeadOrientation: waiting for service: return_joint_states')        
        rospy.wait_for_service("return_joint_states")
        try:
            s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
            resp = s(joint_names)
        except rospy.ServiceException as e:
            rospy.logwarn("%s: Error when calling return_joint_states: %s" % (self._action_name, e))
        for (ind, joint_name) in enumerate(joint_names):
            if(not resp.found[ind]):
                rospy.logwarn("%s: joint %s not found!" % (self._action_name, joint_name ))
        return (resp.position, resp.velocity, resp.effort)

    def get_servo_positions(self):
    
        # EB ROBOT SPECIFIC CODE (DUE TO NECK CONFIGURATION)
        # returns positions of servos. Tilt normalized with neck position (so 0 = facing straight)
        (position, velocity, effort) = self.call_return_joint_states( \
            ['head_sidetilt_joint', 'head_pan_joint', 'head_tilt_joint', 'neck_raise_joint'])

        sidetilt = position[0]
        pan = position[1]
        tilt = position[2]
        neck = position[3]

        #print( " DBG: *** Current Raw Positions: sidetilt = %2.3f, pan = %2.3f, tilt = %2.3f, neck = %2.3f" % 
        #    (sidetilt, pan, tilt, neck))

        # factor in neck positon for tilt
        adjusted_tilt = tilt - neck
        #print( " DBG: *** get_servo_positions raw Tilt = %2.3f, Adjusted Tilt = %2.3f" % (tilt, adjusted_tilt))

        return(sidetilt, pan, adjusted_tilt, neck)

    def get_pan_tilt(self):
        # Return Non-hardware dependent generic pan/tilt
        (servo_sidetilt, servo_pan, servo_tilt, servo_neck) = self.get_servo_positions()
        return(servo_pan, servo_tilt)



class BodyTrackerNode(object):
    def __init__(self):
        super(BodyTrackerNode, self).__init__()

        # init the node
        rospy.init_node('body_tracker', anonymous=False)
        self.log_name = 'body_tracker'
        rospy.loginfo('%s: Starting Node...' % (self.log_name))        
        
        self.HeadOrientation = HeadOrientation()
        self.exit_request = False
        self.yolo_enabled = True # TODO CHANGE TO FALSE?
        self.sort_enabled = False # TODO CHANGE TO FALSE?
        
        # YOLO: load the pre-trained YOLOv8n model
        self.model = YOLO("yolov8n.pt")
        self.tracker = DeepSort(max_age=50)

        # Initialize variables
        self.font=cv2.FONT_HERSHEY_SIMPLEX
        self.scale_display = SCALE_DISPLAY
        self.track_closest_face_only = False

        self.incoming_image_msg = None
        self.incoming_depth_msg = None
        self.cam = None
        self.cv_image = None
        self.cv_window_name = ''
        self._bridge = CvBridge()
        self.loopCount = 0
        self.timeStamp=time.time()
        self.fpsSmooth = 0
        self.smooth_target_x = 0
        self.smooth_target_y = 0
        self.depth_frame_received = False 
        self.color_frame_received = False
        self.skip_frame_count = 0
        
        self.show_cv_debug_window = rospy.get_param("~show_cv_debug_window", True)
        # OVERRIDE for DEBUGGING (if you don't want to mess with the param file) 
        self.show_cv_debug_window = True



        # PUBLISHERS
        body_tracker_array_topic =  '/body_tracker_array/body'
        self.pub_body_tracker_array = rospy.Publisher(body_tracker_array_topic, \
            BodyTrackerArray, queue_size=1)

        self.status_pub = rospy.Publisher('/system_status', SystemStatus, queue_size=8)

        #image_publisher_topic =     '/body_tracker_array/image' # send image synchronized with data
        #self.pub_rgb_image = rospy.Publisher(image_publisher_topic, Image, queue_size=1)


        # SUBSCRIBERS
        rospy.Subscriber("/body_tracker/yolo_enabled", Bool, self.yolo_enabled_callback)
        rospy.Subscriber("/body_tracker/sort_enabled", Bool, self.sort_enabled_callback)

        
        # Subscribe to ROS video message
        camera_rgb_topic  = rospy.get_param("~camera_rgb_topic", "/usb_cam/image_raw")
        self.sub_rgb = rospy.Subscriber(camera_rgb_topic, Image, self.rgb_callback, queue_size=1)
        #self.sub_rgb = rospy.Subscriber(camera_rgb_topic, Image,
        #    self.rgb_callback, queue_size=1, buff_size=2**24)
        rospy.loginfo('%s: Subscribing to camera_rgb_topic: %s' % (self.log_name, camera_rgb_topic))
        rospy.loginfo('%s: Waiting for first frame...' % (self.log_name))

        
        self.send_status_update("BODY_TRACKER", "WAIT_FOR_FRAME")

        # DONE WITH INIT --------------------------------------------------------



    def yolo_enabled_callback(self, data):   
        self.yolo_enabled = data.data
        if self.yolo_enabled:
            rospy.loginfo("%s: Yolo ENABLED" % (self.log_name))
        else:
            rospy.loginfo("%s: Yolo DISABLED" % (self.log_name))

    def sort_enabled_callback(self, data):   
        self.sort_enabled = data.data
        if self.sort_enabled:
            rospy.loginfo("%s: Sort ENABLED" % (self.log_name))
        else:
            rospy.loginfo("%s: Sort DISABLED" % (self.log_name))

        

    def Clamp(self, value, max_value): # clamp between pos and neg max_value
        return max(min(value, max_value), -max_value)

    def send_status_update(self, item, status):
        # Send status update to system monitor
        status_msg = SystemStatus()
        status_msg.item = item
        status_msg.status = status
        self.status_pub.publish(status_msg)

    def shutdown(self):
        # Shuts down the node
        rospy.signal_shutdown("exiting...")


    def rgb_callback(self, data):
        # Callback for RGB images

        if self.skip_frame_count < FRAMES_TO_SKIP:
            self.skip_frame_count = self.skip_frame_count + 1
            return

        self.skip_frame_count = 0        



        #rospy.loginfo("%s: DBG: got rgb_callback! " % (self.log_name))
        #rospy.loginfo("%s: ========= DBG processing new frame ==========================" % (self.log_name))

        try:
            self.incoming_image_msg = data
            # Convert image to numpy array
            self.cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)
            self.cv_image = None

        if self.cv_image is None:
            rospy.loginfo("%s: No frame data, skipping..." %(self.log_name))
            return
        
        # Create the window on the first frame, so we can put it where we want it, but still drag
        if not self.color_frame_received:
            self.color_frame_received = True
            rospy.loginfo('%s: Received first frame. Face Detector Running!' % (self.log_name))                
            self.send_status_update("BODY_TRACKER", "RUNNING")
            
            if self.show_cv_debug_window:
                # print("DBG: displaying CV debug window" )
                self.cv_window_name = 'YOLO Body Tracker'
                cv2.namedWindow(self.cv_window_name)
                #cv2.imshow(self.cv_window_name, self.cv_image)

                if self.scale_display != 1.0:
                    # resize image for display
                    image_height, image_width, image_channels = self.cv_image.shape
                    dim = (int(image_width * self.scale_display), int(image_height * self.scale_display))
                    resized = cv2.resize(self.cv_image, dim, interpolation = cv2.INTER_AREA)
                    cv2.imshow(self.cv_window_name, resized)
                else:
                    cv2.imshow(self.cv_window_name, self.cv_image)

                cv2.moveWindow(self.cv_window_name, 0, 600)
                cv2.waitKey(1) 

        if not self.yolo_enabled:
            rospy.sleep(BODY_TRACKER_SLEEP_TIME)            
            return
            
        # Now, process the frame!            
        self.process_frame()

        # republish the rgb image frame, time synchronized to the data
        #outgoing_rgb_msg = self.incoming_image_msg
        #outgoing_rgb_msg = deepcopy(self.incoming_image_msg)

        # outgoing_rgb_msg.header.stamp = rospy.Time.now()

        #self.pub_rgb_image.publish(outgoing_rgb_msg);
        #print("DGB: Timestamps: Incoming, Outgoing:", self.incoming_image_msg.header.stamp, outgoing_rgb_msg.header.stamp)
        #print("DBG: Timestamps Delta: ", (outgoing_rgb_msg.header.stamp - self.incoming_image_msg.header.stamp))
        #print("DBG: Real Delta: ", (rospy.Time.now() - self.incoming_image_msg.header.stamp))
 
            

        # Allow other ros nodes to run. This Node will take 100% CPU if you let it!
        ## rospy.sleep(BODY_TRACKER_SLEEP_TIME)            
       
        

    def create_body_tracker_msg(self, body_id, 
        scaled_target_radians_x, scaled_target_radians_y,
        base_target_angle_x, base_target_angle_y, target_range_z,
        bb_left, bb_top, bb_width, bb_height, 
        head_pan, head_tilt, name = '', gender = 0, age = 0 ):
    
        body_tracker_msg = BodyTracker()
        
        body_tracker_msg.body_id = body_id
        body_tracker_msg.tracking_status = 0
        body_tracker_msg.gesture = -1 # no gesture
        body_tracker_msg.face_found = False

        body_tracker_msg.bb_left = bb_left
        body_tracker_msg.bb_top = bb_top
        body_tracker_msg.bb_width = bb_width
        body_tracker_msg.bb_height = bb_height
        body_tracker_msg.name = name
        body_tracker_gender = gender
        body_tracker_msg.age = age

        # body center angle x,y from CAMERA center (radians), z = range (meters from camera) 
        body_tracker_msg.camera_to_target_polar.x = scaled_target_radians_x
        body_tracker_msg.camera_to_target_polar.y = scaled_target_radians_y
        body_tracker_msg.camera_to_target_polar.z = target_range_z
 
        # body center angle x,y from ROBOT center front (radians), z = range (meters from camera)
        body_tracker_msg.base_to_target_polar.x = base_target_angle_x
        body_tracker_msg.base_to_target_polar.y = base_target_angle_y
        body_tracker_msg.base_to_target_polar.z = target_range_z

        #body center x,y in camera frame, z = range from camera
        body_tracker_msg.position2d.x = 0.0
        body_tracker_msg.position2d.y = 0.0 
        body_tracker_msg.position2d.z = 0.0

        # body x,y,z in world cartesian coordinates (for point cloud)        
        body_tracker_msg.position3d.x = 0.0 # Point cloud in cartesian coordinates
        body_tracker_msg.position3d.y = 0.0
        body_tracker_msg.position3d.z = 0.0
        
        return(body_tracker_msg)

 
   
    def process_body_info(self, image_height, image_width, head_pan, head_tilt, 
        track_id, ymin, ymax, xmin, xmax):
        
        bb_top =    ymin
        bb_bottom = ymax
        bb_left =   xmin
        bb_right =  xmax

        #print("DBG: before clamp: bb: top = %d, bottom = %d, left = %d, right = %d" % ( bb_top, bb_bottom, bb_left, bb_right))

        # Clamp bb to image (avoid crash with bb that is outside the image)
        bb_top = max(bb_top, 0)
        bb_bottom = min(bb_bottom, image_height)
        bb_left = max(bb_left, 0)
        bb_right = min(bb_right, image_width)

        bb_height = bb_bottom - bb_top
        bb_width = bb_right - bb_left
        
        #print("DBG: clamped bb: top = %d, bottom = %d, left = %d, right = %d" % ( bb_top, bb_bottom, bb_left, bb_right, ))
        #print("DBG: clamped bb: height = %d, width = %d" % ( bb_height, bb_width) )  

        # Generate the target point for servo tracking
        target_point_x = int( bb_left + ((bb_right - bb_left) / 2))
        target_point_y = int( bb_top + ((bb_bottom - bb_top) / 2)) 


        if False: ## TODO REMOVE self.show_cv_debug_window:
            # draw bounding box around the body
            cv2.rectangle(self.cv_image, (bb_left, bb_top), (bb_right, bb_bottom), (0, 255, 0), 4)

        # smooth out the target movement
        undershoot_scale = 0.5
        self.smooth_target_x = (undershoot_scale * self.smooth_target_x) + (undershoot_scale * target_point_x)        
        self.smooth_target_y = (undershoot_scale * self.smooth_target_y) + (undershoot_scale * target_point_y)        

        if self.show_cv_debug_window:
            # Show target point (center of target)
            cv2.rectangle(self.cv_image, (int(self.smooth_target_x-8), int(self.smooth_target_y-8)), 
                                         (int(self.smooth_target_x+8), int(self.smooth_target_y+8)), (255, 255, 255), 4)

            # Debug: Show calculated distance to person
            #item_label =  'eDist: ' +  str(round(estimated_distance, 4)) + '   W: ' + str(bb_width)
            #item_label =  'Width: ' + str(bb_width)
            #cv2.putText(self.cv_image, item_label, (0,90), self.font, 3, (0, 0, 255), 6)


        ######################################################################################
        # convert to radians from center of camera and transpose coordinates (x = left, y = up)
        scaled_target_radians_x = ((self.smooth_target_x / float(image_width)) - 0.5) * FOV_X * -1.0
        scaled_target_radians_y = ((self.smooth_target_y / float(image_height)) - 0.5) * FOV_Y

        camera_x_offset_radians = radians(-9.0)  # degrees. right eye camera is offset from center of face. Compensate here
        scaled_target_radians_x = scaled_target_radians_x + camera_x_offset_radians
        # DEBUG rospy.loginfo( "%s: DBG: *** Target angle from face center: x = %2.1f, y = %2.1f degrees" 
        # DEBUG    % (self.log_name, (scaled_target_radians_x * 57.2958), (scaled_target_radians_y * 57.2958) ))


        # compensate to robot base for head position
        base_target_angle_x  = scaled_target_radians_x + head_pan
        base_target_angle_y  = scaled_target_radians_y + head_tilt
        # DEBUG rospy.loginfo( "%s: DBG: *** Target angle from base center: x = %2.1f, y = %2.1f degrees" 
        # DEBUG    % (self.log_name, (degrees(base_target_angle_x)), (degrees(base_target_angle_y)) ))

        
        # create the body tracker message
        body_id = 0
        if track_id != '':
            try:
                body_id = int(track_id)
            except:
                rospy.logwarn('%s: Bad value [%s] for track_id. Cannot convert to Int' % (self.log_name, track_id))

        estimated_distance = 10000 # no estimated distance
        body_tracker_msg = self.create_body_tracker_msg(
            body_id,
            scaled_target_radians_x, scaled_target_radians_y,
            base_target_angle_x, base_target_angle_y, 
            estimated_distance,
            bb_left, bb_top, bb_width, bb_height, 
            head_pan, head_tilt )

        return body_tracker_msg


    # -------------------------------------------------------------------------
    def process_frame(self):
        # image is in self.cv_image

        person_found = False
        
        # Get the current servo positions to calculate angle compared to body
        head_pan = 0.0
        head_tilt = 0.0
        (head_pan, head_tilt) = self.HeadOrientation.get_pan_tilt()
        # DEBUG rospy.loginfo("%s: Servo Positions:  Pan = %f,  Tilt = %f", self.log_name, head_pan, head_tilt)

        print("---------------------------------------")
        frame_start_time = time.time()
        #frame_start = datetime.datetime.now()


        image_height, image_width, image_channels = self.cv_image.shape
        #print('DBG: Image Shape: ', image_height, image_width, image_channels)

        # mark center of the camera image
        cv2.rectangle(self.cv_image, (int(image_width/2-16), int(image_height/2-16)), 
                                     (int(image_width/2+16), int(image_height/2+16)), 
                                     (255, 255, 255), 4)


        if (self.smooth_target_x == 0) and (self.smooth_target_y == 0):
          # initialize on first frame received
          self.smooth_target_x = image_width / 2 
          self.smooth_target_y = image_height / 2 

       
        # Initialize the Body Tracker Array message
        body_tracker_array_msg = BodyTrackerArray()
        if self.incoming_image_msg:
            # Use same header if responding to a frame from ROS
            body_tracker_array_msg.header = self.incoming_image_msg.header 


        ######################################
        # DETECTION
        ######################################

        # Run the YOLO model on the video frame
        detections = self.model(self.cv_image)[0]

        # initialize the list of bounding boxes and confidences
        results = []

        # loop over the detections
        for data in detections.boxes.data.tolist():
            # extract the confidence (i.e., probability) associated with the prediction
            confidence = data[4]

            #print("DBG: Item in list.")

            # filter out weak detections by ensuring the 
            # confidence is greater than the minimum confidence
            if float(confidence) < CONFIDENCE_THRESHOLD:
                continue

            # if the confidence is greater than the minimum confidence,
            # get the bounding box and the class id
            xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
            class_id = int(data[5])
            class_name = self.model.names[class_id] # Get class name using the ID
            print("Class ID: %d, Name: %s" % (class_id, class_name))            

            # Only add PEOPLE to the results
            if class_id == 0: # people
                print("YOLO Adding Person to list")
                # add the bounding box (x, y, w, h), confidence and class id to the results list
                results.append([[xmin, ymin, xmax - xmin, ymax - ymin], confidence, class_id])

                if not self.sort_enabled:
                    if self.show_cv_debug_window: 
                        # Tracking not enabled, so just show detection bounding box
                        cv2.rectangle(self.cv_image, (xmin, ymin), (xmax, ymax), RED, 2)
                        
                    track_id = '0'
                    body_tracker_msg = self.process_body_info(image_height, image_width, 
                        head_pan, head_tilt, track_id, ymin, ymax, xmin, xmax)
                    body_tracker_array_msg.detected_list.append(body_tracker_msg)
                    person_found = True

            #else:
                #print("YOLO Skipping class_id %d" % class_id)

        yolo_end_time = time.time() # in seconds

            
        ######################################
        # TRACKING
        ######################################

        if not self.sort_enabled:
            print("BodyTracker: Tracking not enabled" )            

        else:
            # update the tracker with the new detections
            tracks = self.tracker.update_tracks(results, frame=self.cv_image)

            # loop over the tracks
            track_id = "0"
            for track in tracks:
                # if the track is not confirmed, ignore it
                if not track.is_confirmed():
                    continue

                # get the track id and the bounding box
                track_id = track.track_id
                
                
                ltrb = track.to_ltrb()

                xmin, ymin, xmax, ymax = int(ltrb[0]), int(
                    ltrb[1]), int(ltrb[2]), int(ltrb[3])
                
            if self.show_cv_debug_window:
                # draw the bounding box and the track id
                cv2.rectangle(self.cv_image, (xmin, ymin), (xmax, ymax), GREEN, 2)
                cv2.rectangle(self.cv_image, (xmin, ymin + 80), (xmin + 100, ymin), BLUE, -1)
                cv2.putText(self.cv_image, track_id, (xmin + 10, ymin + 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 2.0, WHITE, 6)

                body_tracker_msg = self.process_body_info(image_height, image_width, 
                    head_pan, head_tilt, track_id, ymin, ymax, xmin, xmax)
                body_tracker_array_msg.detected_list.append(body_tracker_msg)
                person_found = True



        # Done with all detections
        # Calculate actual processing time
        sort_end_time = time.time()
        yolo_time = yolo_end_time - frame_start_time
        sort_time = sort_end_time - yolo_end_time 
        total_time = sort_end_time - frame_start_time
        # show the time it took to process 1 frame
        print("Compute Time: Yolo: %03.f ms, Sort: %03.f ms, Total = %03.f ms" % (yolo_time*1000, sort_time*1000, total_time*1000))


        #print(f"Time to process 1 frame: {(end - frame_start_time).total_seconds() * 1000:.0f} milliseconds")


           
        # Calculate FPS (includes sleep in calling function)
        dt=time.time()-self.timeStamp
        self.timeStamp=time.time()
        fps=1/dt
        self.fpsSmooth=.9*self.fpsSmooth + .1*fps

        if person_found:

            # At least one person was detected. Publish the body tracker array message
            
            self.pub_body_tracker_array.publish(body_tracker_array_msg)
            # print('DBG: Publishing body_tracker_array message: ')
            # print( body_tracker_array_msg )

        # Show the display window
        if self.show_cv_debug_window:
            
            #print("DBG: displaying CV debug window" )


            flip_mode = 1
            if MIRROR_DISPLAY:
                self.cv_image = cv2.flip(self.cv_image, flip_mode)
            #horz = int(  (image_width * 1.5/2))         
            #horz = int(  (image_width  -50) )        
            horz = int(  (image_width  - 300) )        
            cv2.putText(self.cv_image,str(round(self.fpsSmooth,1))+' fps', (horz, 50 ), self.font, 2.0, (0,255,0), 4)



            if self.scale_display != 1.0:
                # resize image for display
                dim = (int(image_width * self.scale_display), int(image_height * self.scale_display))
                resized = cv2.resize(self.cv_image, dim, interpolation = cv2.INTER_AREA)
                cv2.imshow(self.cv_window_name, resized)

            else:
                cv2.imshow(self.cv_window_name, self.cv_image)

            #elapsed_time = time.time() - frame_start_time # in seconds
            #print( "DBG:Total Frame Elapsed Time = %2.4f" %  (elapsed_time))

            #cv2.moveWindow(self.cv_window_name, 0, 0)
            if cv2.waitKey(1)==ord('q'):
                self.exit_request = True
                self.shutdown()


    def run(self):
        rospy.spin()



def main():
    """ main function
    """
    node = BodyTrackerNode()
    node.run()

if __name__ == '__main__':
    main()
