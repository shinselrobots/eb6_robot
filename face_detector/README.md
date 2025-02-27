# face_detector

ROS node for detecting faces, using mediapipe
Also includes face recognition using a small helper class and python face recognition

Includes sample code from "ros_people_object_detection_tensorflow" by Cagatay Odabasi

References:
- https://github.com/cagbal/ros_people_object_detection_tensorflow

Supports multiple input modes, including exclusive access to a camera or subscribing to an image topic

For realsense, recommended configuration is to run realsense camera configured:
    (modify rs_aligned_depth.launch)
    color: 1920 x 1080
    depth: 1280 x 720
    align_depth = true

camera_rgb_topic: /camera/color/image_raw
optional #camera_depth_topic: /camera/aligned_depth_to_color/image_raw

