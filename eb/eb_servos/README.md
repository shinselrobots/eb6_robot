# eb_servos

## NOTES:
EB Robot uses joystick button 6 (bottom left) to provide head control via joystick, just like TB2S (unlike Sheldon).
TODO: Need to remap messages so this goes through a priority control to mix with other messages such as face tracker?


## Dynamixel setup:
Download Robotis RoboPlus (windows app) from Robotis website
In the RoboPlus Dynamixel Wizard, check all servo limits and set ID and communication speed



# This package is setup for python scripts per ROS standard.
  - launchable python scripts are in ./scripts
  - shared modules for export are in ./src/eb_servos
  - see https://git-amd.tuebingen.mpg.de/aherzog/hinvdyn_example_workspace/blob/master/src/catkin/third_party/catkin/doc/howto/format2/installing_python.rst
  
  
  
  
# NOTE! To get full speed (update_rate > 5), need to reduce serial latency!

    Ubuntu defaults USB serial latency to 16 ms. Need to reduce this to get full dynamixel speed.
    To see ubuntu latency setting: cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

    To see actual ROS update rate:  rostopic echo /diagnostics_dynamixel | grep -A 2 'Update Rate'
    
    Without fix, it's about 5 updates/sec, which sucks if you are trying to monitor servos real-time.
    But with the fix below, and 16 servos at 1Mb/s, measured max rate is ~ 25 updates/sec (40ms), which is pretty good

    This script will fix the problem, but needs root and must be run after each reboot. :-(
        #!/bin/bash
        # ~/.local/bin/fixlatency.sh
        echo -n "Latency timer currently set to: "
        cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer 
        echo -n "Enter password for sudo rights: "
        read -s pass
        echo "$pass" | sudo -S sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'
        echo
        echo -n "Latency timer now set to: "
        cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
        
    Here is a better fix:
    (from: https://unix.stackexchange.com/questions/645744/how-to-edit-latency-timer-without-root-privelege)
    
    First, make a bash script named fix_usb_latency.sh, and move it to /sbin:

        #!/bin/bash
        dev=ttyUSB0
        if [ $# -ge 1 ];then
          dev=$1
        fi
        if [ -f /sys/bus/usb-serial/devices/$dev/latency_timer ] && [ `cat /sys/bus/usb-serial/devices/$dev/latency_timer` -gt 1 ];then
          echo "Fixing the latency issue of $dev..."
          echo 1 | sudo tee /sys/bus/usb-serial/devices/$dev/latency_timer
        fi

    Note: I set the user and group to root, (not sure it's needed):
        gedit fix_usb_latency.sh 
        sudo mv fix_usb_latency.sh /sbin
        cd /sbin
        sudo chmod +x fix_usb_latency.sh 
        sudo chown root fix_usb_latency.sh 
        sudo chgrp root fix_usb_latency.sh 
        ls -la f*
            
    Then, edit /etc/sudoer to let a user or a group execute /sbin/fix_usb_latency.sh without a root privelege.

        $ sudo visudo
        To give a group dialout the permission (% indicates a group):
        %dialout ALL=PASSWD: ALL, NOPASSWD: /sbin/fix_usb_latency.sh

    Then the execution of this script does not request a password.
    Put this into the ubuntu startup applicaitons app, or add a cron job to start at boot:

        $ sudo /sbin/fix_usb_latency.sh ttyUSB0
                


