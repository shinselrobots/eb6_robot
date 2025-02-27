# EB Arduino code
# Also see README in ~/catkin_robot/src/eb/eb/config/udev
# Arduino code in this directory for each of the arduinos in the robot.
# Also, the "mit_app_inventor" directory contains code to run on bluetooth phone
# this code can be modified by uploading from https://appinventor.mit.edu


# Installing Arduino on Linxu

## Quick port check: ls -la /dev | grep "\->"

1. Make sure you configure Arduino as follows:

File-> Preferences -> Sketchbook location: ~/catkin_robot/src/eb/Arduino
  (if you do this, it will automatically include libraries checked in to git)

2.  Check for and Install any missing libraries (compile will show any missing)

    - Tools --> Manage Libraries
    - Adafruit Circuit Playground
    - Adafruit BN0055 or other
    - Adafruit NeoPixel
    - Adafruit Unified Sensor
    - Optinal: Encoder by Paul Stoffregen

3.  Install RosSerial
    sudo apt-get update
    sudo apt-get install ros-noetic-rosserial-arduino
    sudo apt-get install ros-noetic-rosserial

4.  Select the board type depending which Arduino board you are modifying

5.  Set COM Port to port where Arduino is connected



6. See eb_base_arduino_readme.md for setting up Android phone connection


7. Install UDEV RULES from master directory.  See:
   ~/catkin_robot/src/eb/eb/config/udev/README.md

8. Adding custom messages: you need to rerun:
   cd ~/catkin_robot/src/eb/Arduino/libraries
   rm -r ros_lib
   rosrun rosserial_arduino make_libraries.py .
   restart Arduino.
   
9. Troubleshooting:
   if you get serial communication errors, check that you didn't update the wrong arduino with the IDE!
   (yes, I've done that!) Run the following, and check the Arduino IDE to assure it updates the right arduino in the robot!
   ls -la /dev | grep "\->"
   
   If using Feather, make sure you include this at the top!
   #define USE_USBCON
   
   Try running "hello_ros" sketch (in backup folder)
   
   No need to shut down Aduino GUI, but do need to shut down launch when updating from Arduino Gui to avoid serial port conflict.  If it conflicts, just kill and relaunch, does not screw up port.
   

