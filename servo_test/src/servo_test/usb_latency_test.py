#!/usr/bin/env python3

#import sys
import os
import rospy
import logging

def usb_latency_test():

    system_latency = 0
    file_path = '/sys/bus/usb-serial/devices/ttyUSB0/latency_timer'
    if os.path.exists(file_path):
        f = open(file_path, "r")
        latency_str = f.readline()
        f.close()

        #print("[%s]" % latency_str)
        system_latency = int(latency_str)
        #print(system_latency)
        if system_latency == 1:
            print("USB Latency GOOD")
            return(True)
        else:
            rospy.logwarn("USB Latency (%d) BAD! Try running: .local/bin/fixlatency.sh" % (system_latency))
            rospy.logwarn("   or look at '/sys/bus/usb-serial/devices/ttyUSB0/latency_timer")
            rospy.logwarn("   (This usually happens if you unplug usb2dynamixel and forget to reboot)")
            return(False) #sys.exit()
    else:
        rospy.logwarn("latency test: WARNING! No device found on ttyUSB0!")
        return(False) #sys.exit()
   
    
if __name__=='__main__':

    usb_latency_test()
    
        
        
        
    
