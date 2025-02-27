dynamixel_motor
===============

ROS stack for interfacing with Robotis Dynamixel line of servo motors.
This code is cloned from https://github.com/arebgun/dynamixel_motor
It is modified to work with Python3 and tested on ROS Noetic + Ubuntu 20.04 


# NOTE! Ubuntu defaults USB serial latency to 16 ms. Need to reduce this to get full dynamixel speed.
    Typical fastest update without fix is ~5 updates/sec (200ms per update), insufficient 
    if trying to monitor servos real-time.
    
    To see actual update rate:  rostopic echo /diagnostics_dynamixel | grep -A 2 'Update Rate'

    With the fix below, and 16 servos at 1Mb/s, measured max rate is ~ 25 updates/sec (40ms), which is pretty good!

    This script will fix the problem, but needs root and must be run after each reboot. :-(
        #!/bin/bash
        echo -n "Latency timer currently set to: "
        cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer 
        echo -n "Enter password for sudo rights: "
        read -s pass
        echo "$pass" | sudo -S sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'
        echo
        echo -n "Latency timer now set to: "
        cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer


