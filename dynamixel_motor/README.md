dynamixel_motor
===============

ROS stack for interfacing with Robotis Dynamixel line of servo motors.
This code is cloned from https://github.com/arebgun/dynamixel_motor
It is modified to work with Python3 and tested on ROS Noetic + Ubuntu 20.04 

## NOTE! Ubuntu defaults USB serial latency to 16 ms. 
### Need to reduce this to get full dynamixel speed.
Typical fastest update without fix is ~5 updates/sec (200ms per update), 
insufficient if trying to monitor servos real-time.
    
To see actual update rate:  
- rostopic echo /diagnostics_dynamixel | grep -A 2 'Update Rate'

## The fix
See: from: https://unix.stackexchange.com/questions/645744/how-to-edit-latency-timer-without-root-privelege
With the fix below, and 16 servos at 1Mb/s, measured max rate is 
~ 25 updates/sec (40ms), which is pretty good!

## Fix instructions
- First, make a bash script named fix_usb_latency.sh, and move it to /sbin:

        #!/bin/bash
        dev=ttyUSB0
        if [ $# -ge 1 ];then
          dev=$1
        fi
        if [ -f /sys/bus/usb-serial/devices/$dev/latency_timer ] && [ `cat /sys/bus/usb-serial/devices/$dev/latency_timer` -gt 1 ];then
          echo "Fixing the latency issue of $dev..."
          echo 1 | sudo tee /sys/bus/usb-serial/devices/$dev/latency_timer
        fi

- Note: I set the user and group to root, (not sure it's needed):
        gedit fix_usb_latency.sh 
        sudo mv fix_usb_latency.sh /sbin
        cd /sbin
        sudo chmod +x fix_usb_latency.sh 
        sudo chown root fix_usb_latency.sh 
        sudo chgrp root fix_usb_latency.sh 
        ls -la f*
            
- Then, edit /etc/sudoer to let a user or a group execute /sbin/fix_usb_latency.sh without a root privelege.

        $ sudo visudo
        To give a group dialout the permission (% indicates a group):
        %dialout ALL=PASSWD: ALL, NOPASSWD: /sbin/fix_usb_latency.sh

- Then the execution of this script does not request a password.
    - Put this into the ubuntu startup applicaitons app, or add a cron job to start at boot:

        $ sudo /sbin/fix_usb_latency.sh ttyUSB0
                
  
