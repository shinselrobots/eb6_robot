int32  body_id
int32  tracking_status
int32  gesture
bool   face_found

# 2d face bounding Box position in pixels from 0,0 (top left of image)
int32  face_left
int32  face_top
int32  face_width
int32  face_height
int32  age     # rough estimate of persons age
int32  gender  # 0 = unknown, 1 = male, 2 = female
string name    # if match for persons face found in database

# face angle x,y from CAMERA center (radians), z = range (meters) from camera
geometry_msgs/Point32 camera_to_face_polar  

# face angle x,y from ROBOT center (radians), z = range (meters) from camera
geometry_msgs/Point32 base_to_face_polar    

# body x,y,z in world cartesian coordinates (for point cloud)
geometry_msgs/Point32 position3d            

