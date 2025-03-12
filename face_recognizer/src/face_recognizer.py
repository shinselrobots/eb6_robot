#!/usr/bin/env python3
"""
A ROS node to identify known users using face_recognition Python library.
The people who are placed in /people directory will be automatically fetched
and their face features will be compared to incoming face images. If a similar
face is found, the name of the closest face image will be assigned to that
bounding box.
This is modified code from example by:
    Cagatay Odabasi -- cagatay.odabasi@ipa.fraunhofer.de
"""

import rospy
import rospkg
import logging
import message_filters
import glob # for face database
import numpy as np
import cv2

from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image
from body_tracker_msgs.msg import BodyTrackerArray

import face_recognition as fr

# Get the package directory
rospack = rospkg.RosPack()
cd = rospack.get_path('face_recognizer')



# PERFORMANCE GLOBALS:

# To debug, enable publishing a marked-up image showing person(s) found
PUBLISH_DEBUG_IMAGE = True

# Optionally, republish the body_tracker_array message, with names added
REPUBLISH_BODY_TRACKER_ARRAY = True

# To save CPU, only find the person at the center of the image
FIND_CENTER_PERSON_ONLY = True

DELAY_BETWEEN_SEARCHES = 0.1 # 0.10 # seconds to delay between searches to save cpu


class FaceRecognitionNode(object):
    def __init__(self):
        super(FaceRecognitionNode, self).__init__()

        # init the node
        rospy.init_node('face_recognition_node')
        self.log_name = 'Face Recgnition'
        rospy.loginfo("%s: Starting face_recognition_node." % (self.log_name))

        # Get the parameters
        self.show_faces_at_launch = rospy.get_param("~show_faces_at_launch")
        image_topic = rospy.get_param("~camera_topic")
        detection_topic = rospy.get_param("~detection_topic")
        output_topic = rospy.get_param("~output_topic")
        output_topic_rgb = rospy.get_param("~output_topic_rgb")
        output_person_name_topic = rospy.get_param("~output_person_name_topic")

        rospy.loginfo(self.log_name + "Listening on Detection Topic: " + detection_topic)
        rospy.loginfo(self.log_name + "and listening on Image Topic: " + image_topic)

        self.cv_bridge = CvBridge()

        # PUBLISHERS
        self.pub_person_name = rospy.Publisher(output_person_name_topic, String, queue_size=2)

        if REPUBLISH_BODY_TRACKER_ARRAY:
            self.pub_det = rospy.Publisher(output_topic, BodyTrackerArray, queue_size=1)

        if PUBLISH_DEBUG_IMAGE:
            self.pub_det_rgb = rospy.Publisher(output_topic_rgb, Image, queue_size=1)

        # SUBSCRIBERS
        self.sub_detection = message_filters.Subscriber(detection_topic, BodyTrackerArray)
        self.sub_image = message_filters.Subscriber(image_topic, Image)


        # Scaling factor for face recognition image
        self.scaling_factor = 1.0 # 0.50

        # Read the images from folder and create a database
        rospy.loginfo("%s:loading Face Recognition Database..." % (self.log_name))
        self.database = self.initialize_database()

        # Synchronize receipt of image and face bounding box info
        ts = message_filters.ApproximateTimeSynchronizer(\
            [self.sub_detection, self.sub_image], 2, 0.2)
        ts.registerCallback(self.detection_callback)

        rospy.loginfo("%s:Running..." % (self.log_name))

        # spin
        rospy.spin()

        # End of Init
        
        
    def shutdown(self):
        rospy.signal_shutdown("Exiting...")


    def detection_callback(self, detections, image):
        # Callback for synchronized RGB images and detected faces bounding boxes

        # Args: 
        # detections (body_tracker_msgs/BodyTrackerArray) : detections array --> detected_list
        # image (sensor_msgs/Image): RGB image from camera

        #rospy.loginfo("%s:DBG detection_callback." % (self.log_name))

        try:
            cv_rgb = self.cv_bridge.imgmsg_to_cv2(image, "passthrough")[:, :, ::-1]
        except CvBridgeError as e:
            print(e)        

        # cv_rgb = cv2.resize(cv_rgb, (0, 0), fx=self.scaling_factor, fy=self.scaling_factor)

        cv_rgb=cv_rgb.astype(np.uint8)

        # Find Faces and publish results
        self.recognize(detections, cv_rgb)
        


    def find_center_face(self, msg_in, cv_image):
        # Find the face closest to center of the image
        
        image_height, image_width, image_channels = cv_image.shape
        image_center_x = image_width / 2
        image_center_y = image_height / 2
        smallest_delta = 1000
        best_index = 0

        for index, person in enumerate(msg_in.detected_list):
            if person.face_found:
                face_center_x = person.bb_left + person.bb_width / 2
                face_center_y = person.bb_top + person.bb_height / 2

                #print("DBG: Index %d: face_center_x = %d, face_center_y = %d" % 
                #    (index, face_center_x, face_center_y))
                delta_x = abs(image_center_x - face_center_x) 
                delta_y = abs(image_center_y - face_center_y) 
                delta = delta_x + delta_y
                if  delta < smallest_delta:
                    # closest to center so far
                    smallest_delta = delta
                    best_index = index
                      
        #print("DBG: Center Face Index = %d: delta = %d" % (best_index, smallest_delta))

        return(best_index)


    def recognize(self, msg_in, cv_image):

        #rospy.loginfo("%s:DBG recognize" % (self.log_name))
        #print()

        center_face_index = self.find_center_face(msg_in, cv_image)

        center_person_name = ""
        if FIND_CENTER_PERSON_ONLY:
            # For increased speed of name finding and reduced cpu overhead
            person = msg_in.detected_list[center_face_index]
            (center_person_name, cv_image) = self.recognize_face(person, cv_image)
            person.name = center_person_name # edit the message
        
        else:
            # Look for name matches for all the faces. Can take some time...    
            for index, person in enumerate(msg_in.detected_list):
                (person_name, cv_image) = self.recognize_face(person, cv_image)
                person.name = person_name # edit the message
                if index == center_face_index:
                    center_person_name = person_name

        # Publish the name found of person in center, and optionally other stuff
        self.pub_person_name.publish(center_person_name)

        if PUBLISH_DEBUG_IMAGE:
            image_outgoing = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')  # "passthrough")
            self.pub_det_rgb.publish(image_outgoing)

        if REPUBLISH_BODY_TRACKER_ARRAY:
            self.pub_det.publish(msg_in)

        if DELAY_BETWEEN_SEARCHES > 0.0:
            rospy.sleep(DELAY_BETWEEN_SEARCHES)
            

    def recognize_face(self, person, image):

        name_found = ""
        bb_left = bb_top = bb_right =  bb_bottom = 0
        
        if person.face_found:
            #rospy.loginfo("%s:DBG face found, comparing to known faces" % (self.log_name))

            bb_left =   int(  person.bb_left * self.scaling_factor)
            bb_top =    int(  person.bb_top  * self.scaling_factor)
            bb_right =  int( (person.bb_left + person.bb_width) * self.scaling_factor)
            bb_bottom = int( (person.bb_top + person.bb_height) * self.scaling_factor)

            # crop to bounding box
            temp_image = image[bb_top:bb_bottom, bb_left:bb_right ]

            #print("DBG: cropping face to: width = %d, height = %d" %
            #    ( (bb_right - bb_left), (bb_bottom - bb_top)  ))  

            cropped_image_height, cropped_image_width, cropped_image_channels = temp_image.shape
            #print('DBG: Cropped Image Shape: ', cropped_image_height, cropped_image_width, cropped_image_channels)

            # insure there was something in the bounding box
            if temp_image is None or cropped_image_height < 1 or cropped_image_width < 1 or  cropped_image_channels < 3: 
                rospy.loginfo("%s: Bad bounding box, skipping." % (self.log_name))
                print("ERROR: cropping face to: width = %d, height = %d" % ((bb_right-bb_left), (bb_bottom-bb_top)))  

            else:
                rgb_cropped_img = cv2.cvtColor(temp_image, cv2.COLOR_BGR2RGB)

                # Show crop person image - BGR GREEN
                cv2.rectangle(image, (bb_left, bb_top), (bb_right, bb_bottom), (0, 255, 0), 3)

                # DEBUG Show frame size on image - BGR WHITE
                # format: (left, top), (right, bottom)
                cv2.rectangle(image, (30, 10), (838, 470), (255, 255, 255), 2)

                try:


                    face_locations = fr.face_locations(rgb_cropped_img)

                    face_features = fr.face_encodings(rgb_cropped_img, \
                        face_locations)

                    #rospy.loginfo("%s:DBG Running face recognizer..." % (self.log_name))
                    for features, (top, right, bottom, left) in \
                        zip(face_features, face_locations):
                        matches = fr.compare_faces(self.database[0], features)

                        #rospy.loginfo("%s:DBG DBG found a face, looking for matching faces" % 
                        #    (self.log_name))
                        person.name = ""
                        name_label = "Unknown"

                        if True in matches:
                            ind = matches.index(True)
                            name_found = self.database[1][ind]
                            name_label = name_found
                            rospy.loginfo(self.log_name + 
                                "********************************************")
                            rospy.loginfo(self.log_name + 
                                "       FOUND MATCHING FACE! Name = "+ name_found)
                            rospy.loginfo(self.log_name + 
                                "********************************************")
 
                        # Draw bounding boxes on current image
                        l = bb_left + left # map into the face rectangle
                        t = bb_top + top
                        r = bb_left + right
                        b = bb_top + bottom

                        cv2.rectangle(image, (l, t), (r, b), (0, 0, 255), 2) # BGR Red

                        #cv2.rectangle(image, (x, y), \
                        #(x + width, y + height), (255, 0, 0), 3)

                        cv2.putText(image, name_label, \
                        (l + 2, t + 2), \
                        cv2.FONT_HERSHEY_DUPLEX, 1.5, (0, 0, 0), 2)

                        #detections_out.detected_list.append(detection)

                except Exception as e:
                    print(e)
        
        return (name_found, image)  


    def initialize_database(self):
        """
        Reads the PNG images from ./people folder and
        creates a list of peoples

        The names of the image files are considered as their
        real names.

        For example;
        /people
          - mario.png
          - jennifer.png
          - melanie.png

        Returns:
        (tuple) (people_list, name_list) (features of people, names of people)

        """
        filenames = glob.glob(cd + '/people/*.png')

        people_list = []
        name_list = []

        if self.show_faces_at_launch:
            cv2.namedWindow('Face Recognition')
            
        for f in filenames:
            im = cv2.imread(f, 1)

            if self.show_faces_at_launch:
                cv2.imshow('Face Recognition', im)
                cv2.waitKey(50)
                cv2.destroyAllWindows()

            im = im.astype(np.uint8)
            people_list.append(fr.face_encodings(im)[0])
            name_list.append(f.split('/')[-1].split('.')[0])

        return (people_list, name_list)


def main():
    """ main function
    """
    node = FaceRecognitionNode()

if __name__ == '__main__':
    main()
