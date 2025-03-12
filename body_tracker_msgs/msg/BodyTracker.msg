int32  body_id
int32  tracking_status
int32  gesture
bool   face_found

# 2d face or body bounding Box position in pixels from 0,0 (top left of image)
int32  bb_left
int32  bb_top
int32  bb_width
int32  bb_height
int32  age     # rough estimate of persons age
int32  gender  # 0 = unknown, 1 = male, 2 = female
string name    # if match for persons face found in database

# target angle x,y from CAMERA center (radians), z = range (meters) from camera
geometry_msgs/Point32 camera_to_target_polar  

# target angle x,y from ROBOT center (radians), z = range (meters) from camera
geometry_msgs/Point32 base_to_target_polar    

# target center x,y in camera frame, z = range from camera
geometry_msgs/Point32 position2d  

# target x,y,z in world cartesian coordinates (for point cloud)
geometry_msgs/Point32 position3d            

