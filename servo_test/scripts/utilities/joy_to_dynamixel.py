#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

def callback(data):
    if data.buttons[5] == 1:
    	pub_pan.publish(data.axes[2])
    	pub_tilt.publish(data.axes[3])
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()

if __name__ == '__main__':
    pub_pan = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
    pub_tilt = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)
    listener()
