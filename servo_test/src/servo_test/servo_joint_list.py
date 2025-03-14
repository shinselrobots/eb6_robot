#!/usr/bin/env python3
# NOTE: The order here matches servo ID, with the first servo starting at '1'
first_servo_number = 1
last_servo_number = 16

antenna_joints = [
    'right_antenna_joint',
    'left_antenna_joint',
    ]

neck_joints = [
    'head_sidetilt_joint',
    'head_pan_joint',
    'head_tilt_joint',
    'neck_raise_joint',
    ]

left_leg_joints = [
    'left_leg_thigh_lift_joint',
    'left_leg_knee_bend_joint',
    'left_leg_ankle_rotate_joint',
    ]

right_leg_joints = [
    'right_leg_thigh_lift_joint',
    'right_leg_knee_bend_joint',
    'right_leg_ankle_rotate_joint',
    ]
wheel_joints = [
    'left_wheel_motor_joint',
    'right_wheel_motor_joint',
    ]    

head_joints = antenna_joints + neck_joints
all_leg_joints = left_leg_joints + right_leg_joints
leg_joints = all_leg_joints # make it easier and consistant with head
# DONT USE THIS! all_non_wheel_joints = antenna_joints + neck_joints + all_leg_joints
# Wheel joints are NOT included, and must be handled explicitly where needed.
all_servo_joints = antenna_joints + neck_joints + all_leg_joints
all_servo_joints_and_wheels = all_servo_joints + wheel_joints




