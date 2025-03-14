#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

from eb_servos.servo_joint_list import head_joints
from eb_servos.head_servo_publishers import *
from eb_servos.standard_servo_positions import *
from eb_servos.set_servo_speed import *
from eb_servos.set_servo_torque import *
from eb_servos.srv import ReturnJointStates



# ------------------- EB ONLY -------------------------------------------------------------------
# NOTE! For EB, Tilt must be normalized to neck position, which requires some stuff!

# Track current servo positions
def call_return_joint_states(joint_names):
    rospy.wait_for_service("return_joint_states")
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException as e:
        rospy.logwarn("%s: Error when calling return_joint_states: %s" % ("joy_to_dynamixel", e))
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            rospy.logwarn("%s: joint %s not found!" % ("joy_to_dynamixel", joint_name ))
    return (resp.position, resp.velocity, resp.effort)

def get_servo_positions():
    # returns positions of servos. Tilt normalized with neck position (so 0 = facing straight)
    (position, velocity, effort) = call_return_joint_states( \
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


# ------------------- END OF EB ONLY ----------------------------------------------------------------


def callback(data):
    if data.buttons[5] == 1:
        rospy.loginfo("Joystick manually controlling Head!")
        
        tilt_cmd = data.axes[3] # Joystick input (+1.0 to -1.0)
        # print( " Joystick Values: Pan = %2.3f, Tilt = %2.3f" % (data.axes[2], data.axes[3]))
       
# ------------------- EB ONLY -------------------------------------------------------------------
        
        (current_sidetilt, current_pan, current_tilt, current_neck) = get_servo_positions()

        MAX_TILT_UP = 0.52    #  Limit vertical to assure good tracking
        MAX_TILT_DOWN = 1.10  #  max down before hitting neck
        TILT_CENTER = 0.3 #0.15  #  lean toward up (since robot is on the ground)

        new_tilt = current_neck + tilt_cmd + TILT_CENTER
 
        # apply limits
        if new_tilt < -MAX_TILT_DOWN:
            print( "DBG: limiting tilt to MIN. Requested = %2.3f" %  (new_tilt))
            new_tilt = -MAX_TILT_DOWN
        
            
# ------------------- END OF EB ONLY -------------------------------------------------------------------
       
        
        pub_pan.publish(data.axes[2])
        pub_tilt.publish(new_tilt)
            
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()

if __name__ == '__main__':
    pub_pan = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
    pub_tilt = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)
    listener()
