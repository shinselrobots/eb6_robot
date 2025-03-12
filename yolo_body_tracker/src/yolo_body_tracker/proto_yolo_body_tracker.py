#!/usr/bin/env python3

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



import datetime
from ultralytics import YOLO
import cv2
#from helper import create_video_writer
from deep_sort_realtime.deepsort_tracker import DeepSort


CONFIDENCE_THRESHOLD = 0.8
BLUE = (255, 0, 0)
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)


class BodyTrackerNode(object):
    def __init__(self):
        super(BodyTrackerNode, self).__init__()

        # init the node
        rospy.init_node('body_tracker', anonymous=False)
        self.node_name = 'body_tracker'
        rospy.loginfo('%s: Starting Node...' % (self.node_name))        

        self._bridge = CvBridge()

        self.start = datetime.datetime.now()

        # YOLO: load the pre-trained YOLOv8n model
        self.model = YOLO("yolov8n.pt")
        self.tracker = DeepSort(max_age=50)

        # Input Topics
        camera_rgb_topic  = rospy.get_param("~camera_rgb_topic", "/usb_cam/image_raw")
        #camera_rgb_topic  = rospy.get_param("~camera_rgb_topic", "/camera/color/image_raw")
        #camera_rgb_topic  = rospy.get_param("~camera_rgb_topic", "/eye_camera/image_raw")

        #camera_depth_topic  = rospy.get_param("~camera_depth_topic", "")
        #rospy.loginfo('%s: ~camera_depth_topic = %s' % (self.node_name, camera_depth_topic)) 
        #video_file_path = rospy.get_param("~video_file_path", "no")

        # Subscribe to the live ROS video messages
        self.sub_rgb = rospy.Subscriber(camera_rgb_topic,\
            Image, self.rgb_callback, queue_size=1, buff_size=2**24)
        rospy.loginfo('%s: Subscribing to camera_rgb_topic: %s' % (self.node_name, camera_rgb_topic))
        rospy.loginfo('%s: Waiting for first frame...' % (self.node_name))


        # DONE WITH INIT --------------------------------------------------------

    def shutdown(self):
        # Shuts down the node
        cv2.destroyAllWindows() 
        rospy.signal_shutdown("exiting...")


    def rgb_callback(self, data):
        # Callback for RGB images

        #rospy.loginfo("%s: DBG: got rgb_callback! " % (self.node_name))
        #rospy.loginfo("%s: ========= DBG processing new frame ==========================" % (self.node_name))

        try:
            self.incoming_image_msg = data
            # Convert image to numpy array
            self.cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")

            self.process_frame()

        except CvBridgeError as e:
            print(e)

    # -------------------------------------------------------------------------
    def process_frame(self):
        # image is in self.cv_image

        if self.cv_image is None:
            rospy.loginfo("%s: No frame data, skipping..." %(self.node_name))
            return

        frame = self.cv_image
        frame_start = datetime.datetime.now()

        # run the YOLO model on the frame
        detections = self.model(frame)[0]

        # initialize the list of bounding boxes and confidences
        results = []

        ######################################
        # DETECTION
        ######################################

        # loop over the detections
        for data in detections.boxes.data.tolist():
            # extract the confidence (i.e., probability) associated with the prediction
            confidence = data[4]

            # filter out weak detections by ensuring the 
            # confidence is greater than the minimum confidence
            if float(confidence) < CONFIDENCE_THRESHOLD:
                continue

            # if the confidence is greater than the minimum confidence,
            # get the bounding box and the class id
            xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
            class_id = int(data[5])
            # add the bounding box (x, y, w, h), confidence and class id to the results list
            results.append([[xmin, ymin, xmax - xmin, ymax - ymin], confidence, class_id])
            
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), BLUE, 2)
            
        ######################################
        # TRACKING
        ######################################

        if False:
            # update the tracker with the new detections
            tracks = self.tracker.update_tracks(results, frame=frame)
            # loop over the tracks
            for track in tracks:
                # if the track is not confirmed, ignore it
                if not track.is_confirmed():
                    continue

                # get the track id and the bounding box
                track_id = track.track_id
                ltrb = track.to_ltrb()

                xmin, ymin, xmax, ymax = int(ltrb[0]), int(
                    ltrb[1]), int(ltrb[2]), int(ltrb[3])
                # draw the bounding box and the track id
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
                cv2.rectangle(frame, (xmin, ymin - 25), (xmin + 30, ymin), BLUE, -1)
                cv2.putText(frame, (str(track_id)+ "-"), (xmin + 5, ymin - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 2)




        # end time to compute the fps
        end = datetime.datetime.now()
        # show the time it took to process 1 frame
        print(f"Time to process 1 frame: {(end - frame_start).total_seconds() * 1000:.0f} milliseconds")
        # calculate the frame p r second and draw it on the frame
        fps = f"FPS: {1 / (end - self.start).total_seconds():.2f}"
        cv2.putText(frame, fps, (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 8)
        self.start = end            

        # show the frame to our screen
        cv2.imshow("Frame", frame)
        #writer.write(frame)
        if cv2.waitKey(1) == ord("q"):
            return

        rospy.sleep(0.3)
        
        
    def run(self):
        rospy.spin()




def main():
    """ main function
    """
    node = BodyTrackerNode()
    node.run()

if __name__ == '__main__':
    main()




