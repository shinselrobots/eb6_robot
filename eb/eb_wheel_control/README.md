# eb_walk

## Introduction
-  Wheel control for EB robot
-  Also handles Joystick buttons (to allow speed control safety with joystick paddle switch)

-  Receives inputs from:
        joystick:         "/joy" # Standard Logitech / Xbox joystick
        bluetooth phone:  "/phone_joy"
        twist messages:   "/cmd_vel" commands from other modules
        programmed moves: "/simple_move" # Distance (meters), Speed (%), Turn (degrees), Speed (%)
            (see programmed_moves.py)
            
- Publishes Odometry and wheel motor commands:

        "/simple_odom"
        "/simple_odom_right",
        "/simple_odom_left"

        # some additional publishers:
        peek_cmd '/head_peek' from joystick or android? 
        '/behavior/cmd' from joystick buttons
        '/right_wheel_motor_joint/command'
        '/left_wheel_motor_joint/command'


- Also receives inputs from sensors:

        # get imu updates from Arduino BNO086
        "/imu_orientation",

        # monitor battery voltage
        "/battery_voltage"

        # monitor object avoidance sensors
        "/sensor_fusion"



-  Future: control walking steps? (see eb_walk experimental package)



