# UDEV Rules for robot

## Add user to dialout group
    - sudo adduser <your login> dialout
    - sudo reboot now 

## These make sure all devices get mapped correctly
    * To install:
    1. Edit device serial number in usb2dynamixel.rules  
    2. Edit arduino rules if needed to match device installed
       if multiple feather boards (32u4, M0, or M4), you might need to add: KERNELS=="xxxx"
       run: udevadm info -q all -n /dev/ttyACMx | grep DEVPATH to get the KERNELS number.
    3. Run:   sudo ./create_udev_rules.sh


## Normal EB Config should include the following:
    ls -la /dev | grep "\->"
    dynamixel           -> ttyUSBx
    head_arduino        -> ttyACMx
    body_arduino        -> ttyACMx
    

## Ubuntu Latency problem
    Ubuntu defaults to slow serial port (16ms latency). To fix, create following file:
    /sbin/fix_usb_latency.sh
        #!/bin/bash
        # Fix USB latency timer to allow faster access to servos.
        # Copy this file to /sbin
        # NOTE: USB port listed must match the one the device uses!
        dev=ttyUSB0
        if [ $# -ge 1 ];then
          dev=$1
        fi
        if [ -f /sys/bus/usb-serial/devices/$dev/latency_timer ] && [ `cat /sys/bus/usb-serial/devices/$dev/latency_timer` -gt 1 ];then
          echo "Fixing the latency issue of $dev..."
          echo 1 | sudo tee /sys/bus/usb-serial/devices/$dev/latency_timer
        fi

        dev=ttyUSB1
        if [ $# -ge 1 ];then
          dev=$1
        fi
        if [ -f /sys/bus/usb-serial/devices/$dev/latency_timer ] && [ `cat /sys/bus/usb-serial/devices/$dev/latency_timer` -gt 1 ];then
          echo "Fixing the latency issue of $dev..."
          echo 1 | sudo tee /sys/bus/usb-serial/devices/$dev/latency_timer
        fi
    Make sure file has root permisions
    Add to Ubuntu Startup Application
    Test: (if port exists, 1 = good, 16 = bad
        cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
        cat /sys/bus/usb-serial/devices/ttyUSB1/latency_timer

## Arduino
To support AdaFruit boards in Arduine IDE:
    * Open Arduino IDE
    * Preferences -> Additional Boards Manager URLs:
    * Add this: 
        https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
    * Optional:  See this list for more board support options: 
        https://github.com/arduino/Arduino/wiki/Unofficial-list-of-3rd-party-boards-support-urls
    * Install board using the Arduino Board Manager.  More info here:
        https://learn.adafruit.com/adafruit-feather-m4-express-atsamd51/setup
    * For Feather M4 Express (used in Sheldon Arms), install BOTH of these:
        * "Arduino SAMD Boards by Arduino"
        * "Adafruit SAMD Boards by Adafruit"
    * For Feather 32u4 (old), install 
        * "Adafruit AVR Boards by Adafruit"

## Adafruit Feather boards - KERNELS
    Since feather does not have serial number, udev rules have been setup by physical USB port
    See UDEV rules: ~/catkin_robot/src/eb/eb/config/udev

    To get these values, use:
    udevadm info -q all -n /dev/ttyACM0 | grep DEVPATH

                                                                                  >   use this   <    ignore this
    ODOM:   DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-3/1-3.2/.              > 1-3.2:1.0.   <  /tty/ttyACM0

    ARM R:  DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-2/1-2.4/     1-2.4.2/  > 1-2.4.2:1.0  <   /tty/ttyACM0
    ARM L:  DEVPATH=/devices/pci0000:00/0000:00:14.0/usb1/1-2/1-2.4/     1-2.4.1/  > 1-2.4.1:1.0  <   /tty/ttyACM5

## In case of problems:
    https://learn.adafruit.com/adafruit-arduino-ide-setup/linux-setup#udev-rules
    “The rules also fix an issue with ModemManager hanging on to /dev/ttyACM devices”

    1. wget https://github.com/adafruit/Trinket_Arduino_Linux/raw/master/99-adafruit-boards.rules
    2. sudo cp 99-adafruit-boards.rules /etc/udev/rules.d/
    3. Reboot

    Also, this may provide info: tail -f /var/log/syslog | grep tty
    
    Sometimes this helps, but often requires reboot:    
    sudo service udev reload
    sudo service udev restart



