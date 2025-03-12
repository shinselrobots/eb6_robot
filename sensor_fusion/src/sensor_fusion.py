#!/usr/bin/env python3
# combines input from head and body sensors, including laser_scan 
# and other sensor data, and maps to body position

import rospy
import logging
import cv2
import numpy as np
import math

#import message_filters
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from sensor_summary_msgs.msg import SensorSummary
#from body_tracker_msgs.msg import BodyTrackerArray
#import rospkg
from system_status_msgs.msg import SystemStatus


# Get the package directory
#rospack = rospkg.RosPack()
#cd = rospack.get_path('face_recognizer')


class SensorFusion():

    def __init__(self):

        # init the node
        rospy.init_node('sensor_fusion_node')
        rospy.loginfo("Starting sensor_fusion_node...")

        # Get the parameters
        self.show_cv_radar_window = rospy.get_param("~show_cv_radar_window", True)
        self.cv_radar_scale = rospy.get_param("~cv_radar_scale", 150)
        self.cv_window_name = 'Sensor Fusion'
        self.laser_frame_received = False
        self.first_laser_frame_received = False

        # EB is 260mm wide. We pad 50mm on each side to avoid collisions.
        self.robot_width = rospy.get_param("~robot_collision_width", 0.360) 
        self.half_robot_width = self.robot_width / 2
        self.depth_camera_max_range = 2.0 # Camera can actually do > 4.0, but we only care closer than this
        self.avoid_range = rospy.get_param("~object_avoidance_range", 1.20) # 1.20
        self.max_laser_max_range = rospy.get_param("~laser_max_range", 2.00)
        self.ir_max_range = 0.600 # When nothing in view, reports .62 or more
        self.us_max_range = 1.500 # Ultrasonic can actually do > 5.0, but we only care closer than this
        self.ir_left =  2.0
        self.ir_right = 2.0 
        self.us_rear = 2.0

        self.gap_found = False
        self.closest_gap_abs_angle = 0.0
        self.closest_gap_left_angle = 0.0
        self.closest_gap_left_range = 0.0
        self.closest_gap_right_angle = 0.0
        self.closest_gap_right_range = 0.0
        self.closest_gap_size = 0.0
        self.last_obj_angle = 0.0  
        self.last_obj_range = 0.0  
              

        # Subscriber Topics
        laser_scan_topic = rospy.get_param("~laser_topic", "/scan")
        ir_range_topic_left = rospy.get_param("~ir_topic_left", "/sensor_range/ir_left")
        ir_range_topic_right = rospy.get_param("~ir_topic_right", "/sensor_range/ir_right") 
        us_range_topic_rear = rospy.get_param("~us_topic_rear", "/sensor_range/us_rear") 
        
        # Publisher Topics
        output_topic = rospy.get_param("~output_topic", "/sensor_fusion")


        rospy.loginfo("Listening on Laser Topic: " + laser_scan_topic)
        rospy.loginfo("Listening on Left IR Topic: " + ir_range_topic_left)
        rospy.loginfo("Listening on Right IR Topic: " + ir_range_topic_right)

        # PUBLISHERS
        self.sensor_fusion_publisher = rospy.Publisher(output_topic, SensorSummary, queue_size=1)
        self.status_pub = rospy.Publisher('/system_status', SystemStatus, queue_size=6)

        #SUBSCRIBERS
        laser_sub = rospy.Subscriber(laser_scan_topic, LaserScan, self.laser_callback)
        ir_left_sub = rospy.Subscriber(ir_range_topic_left, Float32, self.ir_left_callback)
        ir_right_sub = rospy.Subscriber(ir_range_topic_right, Float32, self.ir_right_callback)
        us_rear_sub = rospy.Subscriber(us_range_topic_rear, Float32, self.us_rear_callback)


        self.send_status_update('DEPTH_CAMERA', "WAITING_FOR_FRAME")
        

    def shutdown(self):
        """
        Shuts down the node
        """
        rospy.signal_shutdown("sensor_fusion_node exiting...")


    def send_status_update(self, item, status):
        # Send status update system monitor
        status_msg = SystemStatus()
        status_msg.item = item
        status_msg.status = status
        self.status_pub.publish(status_msg)


    def ir_left_callback(self, data):
        #rospy.loginfo("DBG: ir_left_callback()")
        #print(data)
        #print('')
        distance = data.data
        self.ir_left  = distance
        #print ("IR_LEFT = ", self.ir_left)


    def ir_right_callback(self, data):
        #rospy.loginfo("DBG: ir_right_callback()")
        #print(data)
        #print('')
        distance = data.data
        self.ir_right = distance
        #print ("IR_RIGHT = ", self.ir_right)

    def us_rear_callback(self, data):
        #rospy.loginfo("DBG: us_rear_callback()")
        #print(data)
        #print('')
        distance = data.data
        self.us_rear = distance
        #print ("US_REAR = ", self.us_rear)


    def angle_to_point(self, angle, distance):
            # Convert angle and distance to cartesian coordinates
            x = distance * math.cos(angle + (-90.0*3.1416/180.0)) # in meters
            y = distance * math.sin(angle + (-90.0*3.1416/180.0)) * -1.0
            return(x,y)


    def measure_gap(self, right_angle, right_range, left_angle, left_range, ):
        # Reached end of a gap. See how big it was, and save the closest to center

        #print("Measure Gap: right angle= %-6.2f, left angle= %-6.2f, right range= %-6.2f, left range= %-6.2f" %
        #    ( math.degrees(right_angle), math.degrees(left_angle), right_range, left_range ) )
        
        x1,y1 = self.angle_to_point(right_angle, right_range)
        x2,y2 = self.angle_to_point(left_angle, left_range)
        
        x_delta = x2-x1
        y_delta = y2-y1
        gapsize = math.sqrt((x_delta * x_delta) + (y_delta * y_delta))
        
        if gapsize > self.robot_width:
            # found a big enough gap. See if it is closer to center than prior gaps
            self.gap_found = True
            min_angle = min(abs(right_angle), abs(left_angle))
            if min_angle < self.closest_gap_abs_angle:
                # Found new closest gap
                self.closest_gap_abs_angle = min_angle
                self.closest_gap_right_angle = right_angle
                self.closest_gap_right_range = right_range
                self.closest_gap_left_angle = left_angle
                self.closest_gap_left_range = left_range
                self.closest_gap_size = gapsize
        
                #print("Closer Gap:     right angle= %-6.2f, left angle= %-6.2f" % 
                #    (math.degrees(self.closest_gap_right_angle), math.degrees(self.closest_gap_left_angle)))
            #else:    
                #print("Not Closer Gap: right angle= %-6.2f, left angle= %-6.2f" % 
                #    (math.degrees(right_angle), math.degrees(left_angle)))


    def laser_callback(self, data):
    
        
        frame = np.zeros((450, 450,3), np.uint8)
        angle_increment = data.angle_increment
        angle = data.angle_min
        base_cv_x = 225
        base_cv_y = 320  # from top of frame
        self.laser_frame_received = True

        # Create the CV window on first frame, so we can put it where we want it, but still drag
        if not self.first_laser_frame_received:
            self.first_laser_frame_received = True
            if self.show_cv_radar_window:
                cv2.namedWindow(self.cv_window_name)
                cv2.imshow(self.cv_window_name, frame)
                cv2.moveWindow(self.cv_window_name, 0, 0)
                cv2.waitKey(1) 
            self.send_status_update('DEPTH_CAMERA', "RUNNING")

       
        i = 0
        measuring_gap = True

        self.gap_found = False
        self.closest_gap_abs_angle = abs(data.angle_min)
        self.closest_gap_left_angle = data.angle_min
        self.closest_gap_left_range = self.max_laser_max_range
        self.closest_gap_right_angle = -data.angle_min
        self.closest_gap_right_range = self.max_laser_max_range
        self.last_obj_angle = data.angle_min  
        self.last_obj_range = self.max_laser_max_range

        # For publishing results        
        sensor_summary = SensorSummary()
        sensor_summary.nearest_object_front = self.max_laser_max_range
        sensor_summary.nearest_object_front_right = self.max_laser_max_range
        sensor_summary.nearest_object_front_left = self.max_laser_max_range
        sensor_summary.nearest_object_side_right = self.max_laser_max_range
        sensor_summary.nearest_object_side_left = self.max_laser_max_range
        sensor_summary.nearest_object_rear = self.max_laser_max_range
        sensor_summary.clear_path_gap_direction = 0.0
        sensor_summary.clear_path_gap_width = self.max_laser_max_range
        sensor_summary.avoidance_range_used = self.avoid_range
 

        # Draw area of interest border right edge
        if self.show_cv_radar_window: 
            border_x, border_y = self.angle_to_point(angle, self.depth_camera_max_range)
            bx = math.trunc(border_x * -self.cv_radar_scale)
            by = math.trunc(border_y * -self.cv_radar_scale)
            cv2.line(frame, (base_cv_x, base_cv_y), (bx+base_cv_x,by+base_cv_y), (255,0,0), 2)

        # Scan from right (angle is neg) to left (angle is pos), zero is in the middle       
        for r in data.ranges:
            object_ahead = False
            object_on_side = False

            # Change infinite values to 0
            if math.isinf(r) == True or math.isnan(r) == True:
                r = self.depth_camera_max_range  # example set to 0.0
                
            # Convert angle and radius to cartesian coordinates
            #x = r * math.cos(angle + (-90.0*3.1416/180.0)) # in meters
            #y = r * math.sin(angle + (-90.0*3.1416/180.0)) * -1.0
            x,y = self.angle_to_point(angle, r)


            # Draw area of interest border
            if self.show_cv_radar_window: 
                #border_x = 0.0
                #border_y = 0.0
                border_x, border_y = self.angle_to_point(angle, self.depth_camera_max_range)
                bx = math.trunc(border_x * -self.cv_radar_scale)
                by = math.trunc(border_y * -self.cv_radar_scale)
            
                cv2.line(frame, (bx+base_cv_x,by+base_cv_y), (bx+base_cv_x,by+base_cv_y+2), (255,0,0), 2)


            # check laser scan for objects ahead.
            if y != 0 and y < self.avoid_range:
                # Object detected
                self.object_found = True
            
                if measuring_gap:
                    self.measure_gap(self.last_obj_angle, self.last_obj_range, angle, r)

                if False: # not i % 10: # just print some of the values for debug
                    print("LASER HIT: % 5d:  angle= %-6.2f, range= %6.2f, x= %-6.2f, y= %6.2f " % 
                    (i, angle, r, x, y))

                if abs(x) < self.half_robot_width:
                    # Object in front of robot
                    object_ahead = True
                    if y < sensor_summary.nearest_object_front:
                        sensor_summary.nearest_object_front = y
                else:
                    # Object to one side
                    object_on_side = True
                    if x < 0:
                        # right side
                        if y < sensor_summary.nearest_object_front_right:
                            sensor_summary.nearest_object_front_right = y
                        if abs(x) < sensor_summary.nearest_object_side_right:
                            sensor_summary.nearest_object_side_right = abs(x)
                    else:
                        # left side
                        if y < sensor_summary.nearest_object_front_left:
                            sensor_summary.nearest_object_front_left = y
                        if abs(x) < sensor_summary.nearest_object_side_left:
                            sensor_summary.nearest_object_side_left = abs(x)
                        

                measuring_gap = False
                self.last_obj_range = r
                self.last_obj_angle = angle
            
            else:
                # no object in avoid range
                measuring_gap = True


            if False: # not i % 50: # print some of the values for debug
                print(" % 5d:  angle= %-6.2f, range= %6.2f, x= %-6.2f, y= %6.2f " % 
                (i, angle, r, x, y))
 
            if self.show_cv_radar_window: 
                draw_x = math.trunc(x * -self.cv_radar_scale)
                draw_y = math.trunc(y * -self.cv_radar_scale)
                          
                # If fill desired, uncomment this:                
                #cv2.line(frame, (base_cv_x, base_cv_y), (draw_x+base_cv_x,draw_y+base_cv_y), (255,0,0), 2)
                
                if object_ahead:
                    cv2.line(frame, (draw_x+base_cv_x,draw_y+base_cv_y), (draw_x+base_cv_x,draw_y+252), (0,100,255), 2)
                
                elif object_on_side:
                    cv2.line(frame, (draw_x+base_cv_x,draw_y+base_cv_y), (draw_x+base_cv_x,draw_y+252), (0,255,255), 2)

                elif y >= self.avoid_range and r < self.depth_camera_max_range - 1:
                    cv2.line(frame, (draw_x+base_cv_x,draw_y+base_cv_y), (draw_x+base_cv_x,draw_y+252), (255, 0, 0), 2)
            
            angle= angle + data.angle_increment 
            i = i+1

              
        # --------------------------------------------------------------------------------
        # Done with all laser scan readings

        # Draw area of interest border left edge
        if self.show_cv_radar_window: 
            border_x, border_y = self.angle_to_point(angle, self.depth_camera_max_range)
            bx = math.trunc(border_x * -self.cv_radar_scale)
            by = math.trunc(border_y * -self.cv_radar_scale)
            cv2.line(frame, (base_cv_x, base_cv_y), (bx+base_cv_x,by+base_cv_y), (255,0,0), 2)

        
        if measuring_gap: # finish any remaining gap measurement
            self.measure_gap(self.last_obj_angle, self.last_obj_range, angle, self.max_laser_max_range)
 
        #if self.ir_left < self.ir_max_range or self.ir_right < self.ir_max_range:
        #    print("IR HIT: left= %-6.2f, right= %-6.2f" % (self.ir_left, self.ir_right)) 

                        
        # summarize gap results
        gap_nav_target_angle = 0.0
        if self.gap_found: 
            # shoot for center of the gap                
            gap_nav_target_angle =  self.closest_gap_right_angle + (
                (self.closest_gap_left_angle - self.closest_gap_right_angle) / 2.0 )

            #print("Final Gap: Target angle= %-6.2f, right angle= %-6.2f, left angle= %-6.2f, gap size= %6.2f" % 
            #    ( math.degrees(gap_nav_target_angle), math.degrees(self.closest_gap_right_angle), 
            #      math.degrees(self.closest_gap_left_angle), self.closest_gap_size ) )

            sensor_summary.clear_path_gap_direction = gap_nav_target_angle
            sensor_summary.clear_path_gap_width = self.closest_gap_size


        else:
            print("GAP SEARCH: No gap found!")
            sensor_summary.clear_path_gap_width = 0.0

        # See if IR sensors detected anything.
        # (IR range values are inaccurate, so don't use to determine object direction)
        if self.ir_right < self.ir_max_range or self.ir_left < self.ir_max_range:
            if self.ir_right < self.ir_max_range and self.ir_left < self.ir_max_range:
                # both IR sensors triggered, so average them
                ir_range = (self.ir_right + self.ir_left) / 2.0
            else:
                ir_range = min(self.ir_right, self.ir_left)
            if ir_range < sensor_summary.nearest_object_front:
                sensor_summary.nearest_object_front = ir_range
            
        # Ultrasonic sensor in the rear
        sensor_summary.nearest_object_rear = self.us_rear

        # Publish summary of all sensor data
        self.sensor_fusion_publisher.publish(sensor_summary)



        # --------------------------------------------------------------------------------
        if self.show_cv_radar_window:

            # Show objects detected by IR sensors 
            if self.ir_left < self.ir_max_range:
                cv_ir_range = int(base_cv_y - self.ir_left * self.cv_radar_scale)
                cv2.line(frame, (base_cv_x-20,cv_ir_range), (base_cv_x-5,cv_ir_range), (0,0,255), 3)
                     
            if self.ir_right < self.ir_max_range:
                cv_ir_range = int(base_cv_y - self.ir_right * self.cv_radar_scale)
                cv2.line(frame, (base_cv_x+20,cv_ir_range), (base_cv_x+5,cv_ir_range), (0,0,255), 3)
 
            # Show objects detected by ultrasonic sensor
            if self.us_rear < self.us_max_range:
                cv_us_range = int(base_cv_y + self.us_rear * self.cv_radar_scale)
                cv2.line(frame, (base_cv_x-20,cv_us_range), (base_cv_x+20,cv_us_range), (0,165,255), 3)
 
            # show avoidance area
            bb_left =   int(base_cv_x - self.half_robot_width * self.cv_radar_scale)
            bb_right =  int(base_cv_x + self.half_robot_width * self.cv_radar_scale)
            bb_bottom = int(base_cv_y)
            bb_top =    int(base_cv_y - self.avoid_range * self.cv_radar_scale)
        
            #cv2.rectangle(frame, (200, 100), (300, 250), (0, 255, 0), 2)
            cv2.rectangle(frame, (bb_left, bb_top), (bb_right, bb_bottom), (0, 255, 0), 1)
            cv2.line(frame, (base_cv_x-self.cv_radar_scale,bb_top), 
                (base_cv_x+self.cv_radar_scale,bb_top), (0,255,0), 1)

            if self.gap_found:
            
                # Show Target Angle
                #gap_nav_target_angle = math.radians(-10.0)
                gap_range = (self.closest_gap_right_range + self.closest_gap_left_range) / 2.0
                x1,y1 = self.angle_to_point(gap_nav_target_angle, gap_range)
                cv2.line(frame, (base_cv_x, base_cv_y),  ( (base_cv_x - int(x1*self.cv_radar_scale)), 
                    (base_cv_y - int(y1*self.cv_radar_scale))),  (255,255,255), 3)

                # Show Gap
                #scale = self.cv_radar_scale
                x1,y1 = self.angle_to_point(self.closest_gap_right_angle, self.closest_gap_right_range)
                x2,y2 = self.angle_to_point(self.closest_gap_left_angle, self.closest_gap_left_range)

                cv2.line(frame, (base_cv_x, base_cv_y),  
                    ( (base_cv_x - int(x1*self.cv_radar_scale)), (base_cv_y - int(y1*self.cv_radar_scale))),
                    (255,127,127), 3) # show right line
                cv2.line(frame, (base_cv_x, base_cv_y), ( (base_cv_x - int(x2*self.cv_radar_scale)), 
                    (base_cv_y - int(y2*self.cv_radar_scale))),  (255,127,127), 3) # show left line
                    
                cv2.line(frame, ((base_cv_x - int(x1*self.cv_radar_scale)), 
                    (base_cv_y - int(y1*self.cv_radar_scale))), ((base_cv_x - int(x2*self.cv_radar_scale)),
                    (base_cv_y - int(y2*self.cv_radar_scale))), (255,127,127), 3) # show gap line

           
            # print("-----------------------------")
                        
            cv2.circle(frame, (base_cv_x, base_cv_y), 2, (255, 255, 255))
            cv2.imshow(self.cv_window_name, frame)
            cv2.waitKey(1)

    def timer_loop(self, event=None):
        # Called every so often to check that depth camera is still sending data
        if self.laser_frame_received:
            self.send_status_update('DEPTH_CAMERA', "RUNNING")        
        else:
            self.send_status_update('DEPTH_CAMERA', "FAIL")
        self.laser_frame_received = False
            


if __name__=='__main__':
    # capture SIGINT signal, e.g., Ctrl+C
    #signal.signal(signal.SIGINT, signal_handler)
    node = SensorFusion()
    rospy.Timer(rospy.Duration(1.0), node.timer_loop) # seconds delay
    rospy.spin() # keep process alive
    
    
    

