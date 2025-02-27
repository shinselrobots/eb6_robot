#!/usr/bin/env python3
"""
ROS node for detecting faces, using mediapipe
Uses some sample code from "ros_people_object_detector_tensorflow" by Cagatay Odabasi,
but code is heavily modified 
"""

import cv2
import sys
import numpy
import time
import csv
import os
from math import radians, degrees

import mediapipe as mp  # Use Mediapipe's face detector


# ROS
import rospy
import logging
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters


from body_tracker_msgs.msg import BodyTracker
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose, Pose2D
from body_tracker_msgs.msg import BodyTrackerArray

from eb_servos.servo_joint_list import head_joints
from eb_servos.srv import ReturnJointStates
from system_status_msgs.msg import SystemStatus

import rospkg
# Get the package directory
rospack = rospkg.RosPack()
cd = rospack.get_path('face_detector')




# GLOBALS / CONSTANTS

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



class FaceDetectorNode(object):
    def __init__(self):
        super(FaceDetectorNode, self).__init__()

        # init the node
        rospy.init_node('face_detector', anonymous=False)
        self.node_name = 'face_detector'
        rospy.loginfo('%s: Starting Node...' % (self.node_name))        
        
        self.HeadOrientation = HeadOrientation()
        self.exit_request = False

        # Get parameters from YAML file

        self.show_cv_debug_window = rospy.get_param("~show_cv_debug_window", True)
        # OVERRIDE for DEBUGGING (if you don't want to mess with the param file) 
        self.show_cv_debug_window = True

         
        # Input Topics
        camera_rgb_topic  = rospy.get_param("~camera_rgb_topic", "/usb_cam/image_raw")
        #camera_rgb_topic  = rospy.get_param("~camera_rgb_topic", "/camera/color/image_raw")
        #camera_rgb_topic  = rospy.get_param("~camera_rgb_topic", "/eye_camera/image_raw")

        camera_depth_topic  = rospy.get_param("~camera_depth_topic", "")
        rospy.loginfo('%s: ~camera_depth_topic = %s' % (self.node_name, camera_depth_topic)) 
        video_file_path = rospy.get_param("~video_file_path", "no")


        # TODO Output Topics - set publisher topics here
        #body_tracker_topic =        '/body_tracker/position'
        body_tracker_array_topic =  '/body_tracker_array/people'
        image_publisher_topic =     '/body_tracker_array/image' # send image synchronized with data

 
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
        
       
        
        # Mediapipe face detecton
        mp_long_range_mode = True
        mp_confidence_threshold = 0.7
        self.mp_FaceDetector = mp.solutions.face_detection.FaceDetection( 
            model_selection=mp_long_range_mode, min_detection_confidence=mp_confidence_threshold)
        self.draw_utils = mp.solutions.drawing_utils        


        # PUBLISHERS
        # Advertise the BodyTrackerArray message (same as Nuitrack node, but an array!)
        self.pub_body_tracker_array = rospy.Publisher(body_tracker_array_topic, \
            BodyTrackerArray, queue_size=1)
        self.status_pub = rospy.Publisher('/system_status', SystemStatus, queue_size=8)

        # Advertise the image frame message. We republish the image at 
        # the same time as the data, to assure they are synchronized.
        self.pub_rgb_image = rospy.Publisher(image_publisher_topic, Image, queue_size=1)



        # Subscribe to the live ROS video messages

        if False: # TODO DEBUG DISABLED    camera_depth_topic:
            # When depth is specified, synchronize RGB and Depth frames
            # warning!  if used, but no depth camera, RGB will never show up!

            # Subscribe to approx synchronized rgb and depth frames
            self.sub_rgb = message_filters.Subscriber(camera_rgb_topic, Image)
            self.sub_depth = message_filters.Subscriber(camera_depth_topic, Image)

            # Create the message filter
            ts = message_filters.ApproximateTimeSynchronizer(\
                [self.sub_rgb, self.sub_depth], 2, 0.9)
            ts.registerCallback(self.rgb_and_depth_callback)
            
            rospy.logwarn(self.node_name + "Subscribing to SYNCHRONIZED RGB: " + \
            camera_rgb_topic + " and Depth: " + camera_depth_topic)
            rospy.logwarn("%s: WILL WAIT FOREVER FOR DEPTH FRAME!" % (self.node_name))

        else:
            # no depth topic, RGB only

            self.sub_rgb = rospy.Subscriber(camera_rgb_topic,\
                Image, self.rgb_callback, queue_size=1, buff_size=2**24)
            rospy.loginfo('%s: Subscribing to camera_rgb_topic: %s' % (self.node_name, camera_rgb_topic))
            rospy.loginfo('%s: Waiting for first frame...' % (self.node_name))

        self.send_status_update("FACE_DETECTOR", "WAIT_FOR_FRAME")

        # DONE WITH INIT --------------------------------------------------------
        

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


    def rgb_and_depth_callback(self, rgb_msg, depth_msg):
        """
        Callback for synchronized RGB and Depth frames
        Allows distance to people to be determined 
        """
        if not self.depth_frame_received:
            self.depth_frame_received = True
            rospy.logwarn("%s: Got rgb_and_depth_callback! DEPTH FRAME RECEIVED OK!")

        # save the depth image
        self.incoming_depth_msg = depth_msg

        # call the rgb frame handler as usual
        self.rgb_callback(rgb_msg)
        


    def rgb_callback(self, data):
        # Callback for RGB images

        #rospy.loginfo("%s: DBG: got rgb_callback! " % (self.node_name))
        #rospy.loginfo("%s: ========= DBG processing new frame ==========================" % (self.node_name))

        try:
            self.incoming_image_msg = data
            # Convert image to numpy array
            self.cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Create the window on the first frame, so we can put it where we want it, but still drag
            if not self.color_frame_received:
                self.color_frame_received = True
                rospy.loginfo('%s: Received first frame. Face Detector Running!' % (self.node_name))                
                self.send_status_update("FACE_DETECTOR", "RUNNING")
                
                if self.show_cv_debug_window:
                    # print("DBG: displaying CV debug window" )
                    self.cv_window_name = 'Face Detector'
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
            
            self.process_frame()

            # republish the rgb image frame, time synchronized to the data
            outgoing_rgb_msg = self.incoming_image_msg
            
            #outgoing_rgb_msg = deepcopy(self.incoming_image_msg)
            
            #rospy.sleep(1.)
            #print("DBG: Current time = ", rospy.Time.now() )
 
            outgoing_rgb_msg.header.stamp = rospy.Time.now()
 
            self.pub_rgb_image.publish(outgoing_rgb_msg);
            #print("DGB: Timestamps: Incoming, Outgoing:", self.incoming_image_msg.header.stamp, outgoing_rgb_msg.header.stamp)
            #print("DBG: Timestamps Delta: ", (outgoing_rgb_msg.header.stamp - self.incoming_image_msg.header.stamp))
            #print("DBG: Real Delta: ", (rospy.Time.now() - self.incoming_image_msg.header.stamp))
 
            
        except CvBridgeError as e:
            print(e)

       
        

    def create_body_tracker_msg(self, body_id, 
        scaled_target_radians_x, scaled_target_radians_y,
        base_target_angle_x, base_target_angle_y, target_range_z,
        bb_left, bb_top, bb_width, bb_height, 
        head_pan, head_tilt, name = '', gender = 0, age = 0 ):
    
        body_tracker_msg = BodyTracker()
        
        body_tracker_msg.body_id = body_id
        body_tracker_msg.tracking_status = 0
        body_tracker_msg.gesture = -1 # no gesture
        body_tracker_msg.face_found = True

        body_tracker_msg.face_left = bb_left
        body_tracker_msg.face_top = bb_top
        body_tracker_msg.face_width = bb_width
        body_tracker_msg.face_height = bb_height
        body_tracker_msg.name = name
        body_tracker_gender = gender
        body_tracker_msg.age = age

        # face angle x,y from CAMERA center (radians), z = range (meters from camera) 
        body_tracker_msg.camera_to_face_polar.x = scaled_target_radians_x
        body_tracker_msg.camera_to_face_polar.y = scaled_target_radians_y
        body_tracker_msg.camera_to_face_polar.z = target_range_z
 
        # face angle x,y from ROBOT center front (radians), z = range (meters from camera)
        body_tracker_msg.base_to_face_polar.x = base_target_angle_x
        body_tracker_msg.base_to_face_polar.y = base_target_angle_y
        body_tracker_msg.base_to_face_polar.z = target_range_z
        
        body_tracker_msg.position3d.x = 0.0 # Point cloud in cartesian coordinates
        body_tracker_msg.position3d.y = 0.0
        body_tracker_msg.position3d.z = 0.0
        
        return(body_tracker_msg)

   
    def distance_to_face(self, image_width, face_pixel_width ):
        # use triangle similarity to estimate distance
        # Good writeup: https://pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
        # Calibration of a new camera: F = (P*D) / W where F = Focal length, P = Pixels, D = Distance, W = Known Object Width
        # Finding Distance: D = (W*F) / P
        cam_focal_length = CAMERA_FOCAL_SCALE * float(image_width) # scale to resolution that is streaming
        #cam_focal_length = REALSENSE_RGB_FOCAL_LENGTH
        # TODO - auto switch this based upon which frame we are processing!
        
        distance = (AVG_FACE_WIDTH * cam_focal_length) / face_pixel_width 
        
        return(distance)



    # -------------------------------------------------------------------------
    def process_frame(self):
        # image is in self.cv_image

        if self.cv_image is None:
            rospy.loginfo("%s: No frame data, skipping..." %(self.node_name))
            return

        #frame_start_time = time.time()


        image_height, image_width, image_channels = self.cv_image.shape
        #print('DBG: Image Shape: ', image_height, image_width, image_channels)
        if (self.smooth_target_x == 0) and (self.smooth_target_y == 0):
          # initialize on first frame received
          self.smooth_target_x = image_width / 2 
          self.smooth_target_y = image_height / 2 

        bodyID = 1
        detected_face_index = 0
       
        # Initialize the Body Tracker Array message
        body_tracker_array_msg = BodyTrackerArray()
        if self.incoming_image_msg:
            # Use same header if responding to a frame from ROS
            body_tracker_array_msg.header = self.incoming_image_msg.header 


        # run face detector
        frameRGB = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)

        results = self.mp_FaceDetector.process(frameRGB)
        #print( results.detections )
        
        if results.detections != None:
        
            #detect_elapsed_time = time.time() - frame_start_time # in seconds
            #print( "DBG:DETECT Elapsed Time = %2.4f" %  (detect_elapsed_time))
        

            # DEPTH IS DISABLED FOR THIS ROBOT
            if False: 
                # get depth message (if any)
                cv_depth_image = None
                cv_depth_image_received = False
                if self.incoming_depth_msg:
                    # Convert image to numpy array
                    cv_depth_image = self._bridge.imgmsg_to_cv2(self.incoming_depth_msg, "passthrough")
                    cv_depth_image_received = True
                    #rospy.loginfo("DBG GOT incoming_depth_msg ")

            # Get the current servo positions to calculate angle compared to body
            head_pan = 0.0
            head_tilt = 0.0
            (head_pan, head_tilt) = self.HeadOrientation.get_pan_tilt()
            # DEBUG rospy.loginfo("%s: Servo Positions:  Pan = %f,  Tilt = %f", self.node_name, head_pan, head_tilt)

            # Loop through all faces detected        
            for face in results.detections:
            
                #print('Label ID:   ', face.label_id)
                #print('Confidence: ', face.score)
                label_id = int(face.label_id[0])
                confidence_score = float(face.score[0])
                #print('Label ID:   ', label_id)
                #print('Found Face Confidence: ', round(confidence_score,2) )
                
                self.draw_utils.draw_detection(self.cv_image, face)
      
                detected_face_index += 1
                rbox = face.location_data.relative_bounding_box #values are -1.0 to +1.0 (percent of frame from center)
                # convert to pixel coordinates
                bb_top =    int(rbox.ymin * image_height)
                bb_bottom = int( (rbox.ymin+rbox.height) * image_height)
                bb_left =   int(rbox.xmin * image_width)
                bb_right =  int( (rbox.xmin+rbox.width)  * image_width)

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
                target_point_y = int( bb_top + ((bb_bottom - bb_top) / 2)) # if tilted realsense, div by 4 to shift to eyes
                # TODO consider using: mp_face_detection.FaceKeyPoint.NOSE_TIP from 
                # https://mediapipe.readthedocs.io/en/latest/solutions/face_detection.html

                # Estimate distance to person based upon size of face bounding box
                estimated_distance = self.distance_to_face(image_width, bb_width)
                # print('bb_width, estimated_distance = ', bb_width, estimated_distance)

                if self.show_cv_debug_window:
                    # draw bounding box around the face
                    cv2.rectangle(self.cv_image, (bb_left, bb_top), (bb_right, bb_bottom), (0, 255, 0), 2)

                # smooth out the target movement
                undershoot_scale = 0.5
                self.smooth_target_x = (undershoot_scale * self.smooth_target_x) + (undershoot_scale * target_point_x)        
                self.smooth_target_y = (undershoot_scale * self.smooth_target_y) + (undershoot_scale * target_point_y)        

                if self.show_cv_debug_window:
                    cv2.rectangle(self.cv_image, (int(self.smooth_target_x-8), int(self.smooth_target_y-8)), 
                                                 (int(self.smooth_target_x+8), int(self.smooth_target_y+8)), (255, 255, 255), 4)

                    cv2.rectangle(self.cv_image, (int(image_width/2-16), int(image_height/2-16)), 
                                                 (int(image_width/2+16), int(image_height/2+16)), (255, 255, 255), 4)

                    # Debug: Show calculated distance to person
                    item_label =  'eDist: ' +  str(round(estimated_distance, 4)) + '   W: ' + str(bb_width)
                    #item_label =  'Width: ' + str(bb_width)
                    cv2.putText(self.cv_image, item_label, (0,90), self.font, 3, (0, 0, 255), 6)


                ######################################################################################
                # convert to radians from center of camera and transpose coordinates (x = left, y = up)
                scaled_target_radians_x = ((self.smooth_target_x / float(image_width)) - 0.5) * FOV_X * -1.0
                scaled_target_radians_y = ((self.smooth_target_y / float(image_height)) - 0.5) * FOV_Y

                camera_x_offset_radians = radians(-9.0)  # degrees. right eye camera is offset from center of face. Compensate here
                scaled_target_radians_x = scaled_target_radians_x + camera_x_offset_radians
                # DEBUG rospy.loginfo( "%s: DBG: *** Target angle from face center: x = %2.1f, y = %2.1f degrees" 
                # DEBUG    % (self.node_name, (scaled_target_radians_x * 57.2958), (scaled_target_radians_y * 57.2958) ))


                # compensate to robot base for head position
                base_target_angle_x  = scaled_target_radians_x + head_pan
                base_target_angle_y  = scaled_target_radians_y + head_tilt
                # DEBUG rospy.loginfo( "%s: DBG: *** Target angle from base center: x = %2.1f, y = %2.1f degrees" 
                # DEBUG    % (self.node_name, (degrees(base_target_angle_x)), (degrees(base_target_angle_y)) ))

                
                # create the body tracker message
                body_tracker_msg = self.create_body_tracker_msg(detected_face_index,
                    scaled_target_radians_x, scaled_target_radians_y,
                    base_target_angle_x, base_target_angle_y, estimated_distance,
                    bb_left, bb_top, bb_width, bb_height, head_pan, 
                    head_tilt )

                body_tracker_array_msg.detected_list.append(body_tracker_msg)


        # Done with all detections
           
        # Calculate FPS
        dt=time.time()-self.timeStamp
        self.timeStamp=time.time()
        fps=1/dt
        self.fpsSmooth=.9*self.fpsSmooth + .1*fps

        if detected_face_index > 0:

            # At least one face was detected. Publish the body tracker array message
            
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
            horz = int(  (image_width  - 300) )        
            cv2.putText(self.cv_image,str(round(self.fpsSmooth,1))+' fps', (horz, 50 ), self.font, 1.5, (0,255,0), 2)

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
    node = FaceDetectorNode()
    node.run()

if __name__ == '__main__':
    main()
