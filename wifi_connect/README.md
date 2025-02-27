# wifi_connect and boot
##  This package enables booting a headless (no monitor) robot on an new wifi 
##  network (such as when doing demos at a new location)
##  Example dependency files are included, but must be modified for your robot



## Hardware:
This package assumes the following hardware configuration:
- An Android bluetooth phone, running an app created with MIT App Inventor:
  https://appinventor.mit.edu/

- A bluetooth connection between the phone and the ROS computer on the robot
  I use an arduino, with a "BlueSMiRF" module or similar

## Software Overview
- MIT App Inventor app installed on Bluetooth Phone 
- Arduino code on robot arduino
- Startup Code to launch bootup ROS modules

## Installation
- Install this package

- Install MIT App Inventor app on Bluetooth Phone (see instructions at https://appinventor.mit.edu/)
    see example .aia file in examples

- Modify and copy Arduino code for your robot
    see .ino file in examples

- Modify and copy example boot shell scripts:
    Edit the two ./examples/robot_boot*.sh files to point to your ROS launch files
        edit ./examples/robot_boot1.sh
        edit ./examples/robot_boot1.sh
    
    Then copy them to your .local/bin directory:
        cp ./examples/robot_boot*.sh ~/.local/bin/
        chmod a+x ~/.local/bin/robot_boot*.sh

- Modify and copy example ROS Launch files:
    edit ./examples/robot_boot1.launch <-- This launches the arduino and wifi_connect nodes
    edit ./examples/robot_boot2.launch <-- This will be your main launch file for your robot, without the arduino

- Edit wifi_connect.py to point to <your home>/.local/bin directory

- Edit "Startup Applications" (or whatever your linux uses) to include the following:
    gnome-terminal -- bash -c "/home/<your_login>/.local/bin/launch_robot_booter.sh; exec bash"



