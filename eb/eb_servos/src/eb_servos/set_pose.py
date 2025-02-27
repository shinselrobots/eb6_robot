#!/usr/bin/env python3

import sys
import roslib
# roslib.load_manifest('eb_servos')
import rospy, time

# from eb_servos.standard_servo_positions import *
from eb_servos.leg_servo_publishers import *
from eb_servos.head_servo_publishers import *

from eb_servos.set_servo_speed import *
from eb_servos.set_servo_torque import *
from eb_servos.servo_joint_list import *
from eb_servos.srv import ReturnJointStates

# Constants (Knee detection value for each pose)
DETECT_ANGLE_POSE1 = 2.14
DETECT_ANGLE_POSE2 = 1.77
DETECT_ANGLE_POSE3 = 1.48
DETECT_ANGLE_POSE4 = 0.9
MAX_POSE = 5
REST_POSE = 6 # REST pose after moving servos into position 0 or 1.

TABLE_MODE_HEAD_TILT_OFFSET = 0.0
FLOOR_MODE_HEAD_TILT_OFFSET = 0.500
MAGNET_DISENGAGE_SPEED = 1.5 # Move servo fast to pop it on or off the magnet

class RobotPose():

    def __init__(self, name):
        self._calling_name = name
        self._current_pose = -1 # Not init yet. Valid pose is 0 - 5 (sleep, sit, mid-low, mid (ready), mid-high, high)

       
        # WARNING - Must set DETECT_ANGLE_POSE if you change these leg values!
        # ALSO, must keep these three tables in sync!
        # Servo position for: 
        #      thigh,  knee,  ankle, neck_raise, head_tilt, antennas
        self._pos_lookup = \
            [
            [ -0.745, 2.335, -1.500, -1.600, -1.470,  1.500],  # pos 0 (sleep)

            [ -0.790, 2.390, -1.500, -0.700, -0.400, -0.600],  # pos 1 (sit)   Note: neck up for face tracking
            [ -0.750, 1.950, -1.200, -0.800, -0.600, -0.600],  # pos 2 (mid-low) 
            [ -0.700, 1.571, -0.940, -0.500, -0.400, -0.600],  # pos 3 (mid / ready) -- knee at 45 degrees
            [ -0.500, 1.200, -0.750, -0.300, -0.200, -0.400],  # pos 4 (mid-high)
            [ -0.200, 0.400, -0.330, -0.000, -0.100, -0.000],  # pos 5 (high)

            [ -0.750, 2.335, -1.480, -1.470, -1.470,  1.500]]  # pos 6 - REST pose



        self._lean_lookup = \
            [
            [ -0.745, 2.335, -1.500, -1.600, -1.470,  1.500],  # pos 0 (sleep)
            [ -0.790, 2.390, -1.500, -0.700, -0.400, -0.600],  # pos 1 (sit)   Note: neck up for face tracking
            [ -0.750, 1.950, -1.200, -0.800, -0.600, -0.600],  # pos 2 (mid-low) 
                       
            [ -0.710, 1.620, -0.970, -0.500, -0.400, -0.600],  # pos 3 (mid / ready) 1/8 distance
          # [ -0.720, 1.666, -1.005, -0.500, -0.400, -0.600],  # pos 3 (mid / ready) 1/4 distance - save this one


            
            [ -0.500, 1.200, -0.750, -0.300, -0.100, -0.400],  # pos 4 (mid-high)
            [ -0.200, 0.400, -0.330, -0.000,  0.200, -0.000],  # pos 5 (high)
            [ -0.750, 2.335, -1.480, -1.470, -1.470,  1.500]]  # pos 6 - REST pose



            
        # Speed multiplier for each servo to move synchronously
        self._speed_mul = [
            [[1.00, 2.00, 1.50],  # from pos 0 (SLEEP) to pos 0-5, thigh, knee, ankle
            [ 1.00, 2.00, 1.50],  # to 1
            [ 0.60, 2.20, 1.50],  # to 2
            [ 0.40, 2.50, 1.90],  # to 3
            [ 0.60, 2.60, 1.90],  # to 4 
            [ 0.70, 2.50, 1.50]], # to 5

            [[1.00, 2.00, 1.50],  # from pos 1 (SIT) to pos 0-5, thigh, knee, ankle
            [ 1.00, 2.00, 1.50],  # <--- to same
            [ 0.60, 2.20, 1.50],  # to 2
            [ 0.40, 2.50, 1.80],  # to 3
            [ 0.60, 2.60, 1.80],  # to 4 
            [ 0.70, 2.50, 1.50]], # to 5

            [[1.00, 2.00, 1.50],  # from pos 2 (MID-LOW)to pos 0-5, thigh, knee, ankle
            [ 1.00, 2.00, 1.50],  # to 1
            [ 1.00, 2.00, 1.50],  # <--- to same
            [ 1.00, 2.30, 1.60],  # to 3
            [ 1.00, 2.80, 1.70],  # to 4
            [ 0.90, 2.40, 1.30]], # to 5

            [[0.40, 2.80, 1.80],  # ***  from pos 3 (MID / READY) to pos 0-5, thigh, knee, ankle
            [ 0.40, 2.80, 1.80],  # to 1
            [ 1.00, 2.00, 1.30],  # to 2
            [ 1.10, 2.30, 1.35],  # <--- to same
            [ 1.20, 2.60, 1.40],  # to 4
            [ 1.00, 2.25, 1.20]], # to 5

            [[1.20, 2.80, 1.70],  # from pos 4 (MID-HIGH) to pos 0-5, thigh, knee, ankle
            [ 1.20, 2.80, 1.70],  # to 1
            [ 1.00, 2.80, 1.70],  # to 2
            [ 1.30, 2.70, 1.30],  # to 3
            [ 1.20, 2.60, 1.20],  # <--- to same
            [ 1.00, 2.10, 1.10]], # to 5

            [[1.10, 2.80, 1.50],  # from pos 5 (HIGH) to pos 0-5, thigh, knee, ankle
            [ 1.10, 2.80, 1.60],  # to 1
            [ 1.20, 2.70, 1.50],  # to 2
            [ 1.10, 2.70, 1.40],  # to 3
            [ 1.00, 2.40, 1.30],  # to 4
            [ 1.00, 2.40, 1.30]]] # <--- to same


    def get_ankle_neutral_position(self, pose):
        ankle_position = self._pos_lookup[pose][2]
        return ankle_position
        

    def get_initial_pose(self):
        # Get current position of knee servo to determine intial pose of robot,
        # since robot might be in any state from a prior run.
        
        while True:
            (position, velocity, effort) = self.call_return_joint_states(['right_leg_knee_bend_joint'])
            if position[0] == 0.0:  
                # a real servo is never exactly 0.0!
                rospy.logwarn("%s: waiting for right_leg_knee_bend_joint servo valid position, to initalize starting pose! Ready = %d" % (self._calling_name, ready[0]))
            else:
                # print( "DBG: set_pose: right_leg_knee_bend_joint servo ready. Value = %2.3f Getting Initial Pose." % (position[0]))
                break
       
        right_leg_knee_bend_pos = position[0]
        if right_leg_knee_bend_pos > DETECT_ANGLE_POSE1:
            self._current_pose = 1 # could be 0 or 1, which have same leg pose (current head pose not important for this)
        elif right_leg_knee_bend_pos > DETECT_ANGLE_POSE2:
            self._current_pose = 2
        elif right_leg_knee_bend_pos > DETECT_ANGLE_POSE3:
            self._current_pose = 3
        elif right_leg_knee_bend_pos > DETECT_ANGLE_POSE4:
            self._current_pose = 4
        else:
            self._current_pose = 5

        rospy.loginfo("%s: Initial Pose is %d (knee position = %2.3f) " % (self._calling_name, self._current_pose, right_leg_knee_bend_pos))
   
        return self._current_pose


    def call_return_joint_states(self, joint_names):
        rospy.wait_for_service("return_joint_states")
        try:
            s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
            resp = s(joint_names)
        except rospy.ServiceException as e:
            rospy.logwarn("%s: Error when calling return_joint_states: %s" % (self._calling_name, e))
        for (ind, joint_name) in enumerate(joint_names):
            if resp.found == 0:
                rospy.logwarn("%s: joint %s not found!" % (self._calling_name, joint_name ))
        return (resp.position, resp.velocity, resp.effort)

    def get_current_pose(self):
        # TODO this is ERROR - different code has copies of this file, they are not synced!
        ## ERROR: if self._current_pose < 0:
        # initial pose not set yet.
        self.get_initial_pose()            
        return  self._current_pose
        
    def pose_down(self, lowest_servo_speed=0.3, head_center=True):
        current_pose = self.get_current_pose()
        new_pose = current_pose - 1
        if new_pose < 0:
            rospy.logwarn("%s: Robot at MIN POSE. pose_down Ignored." % (self._calling_name))
        elif new_pose == 0:
            self.move(new_pose, lowest_servo_speed, True) # Force head center for sleep pose
        else:
            self.move(new_pose, lowest_servo_speed, head_center)

    def pose_up(self, lowest_servo_speed=0.3, head_center=True):
        current_pose = self.get_current_pose()
        new_pose = current_pose + 1
        if new_pose > MAX_POSE:
            rospy.logwarn("%s: Robot at MAX POSE. pose_up Ignored." % (self._calling_name))
        else:
            self.move(new_pose, lowest_servo_speed, head_center)

    def lean_calc(self, current_pose, lean_amount, servo):
        current_pose_value =  self._pos_lookup[current_pose][servo]  
        lower_pose_value = self._pos_lookup[(current_pose-1)][servo]
        if lean_amount == 0:
            return current_pose_value
        lean_value = current_pose_value + ((lower_pose_value - current_pose_value) * lean_amount)
        return lean_value
        
    # =================================================================================================
    def lean2(self, direction, lean_amount=0.12, lowest_servo_speed=0.3):
    # lean amount is percent toward lower pose. 0.0 = current pose, 1.0 = lower pose
   
        base_pose = self.get_current_pose()

        # Get servo neutral goal positions
        thigh_neutral     = self._pos_lookup[base_pose][0]
        knee_neutral      = self._pos_lookup[base_pose][1]        
        ankle_neutral     = self._pos_lookup[base_pose][2]

        # Get servo lean goal positions
        thigh_lean = self.lean_calc(base_pose, lean_amount, 0)
        knee_lean = self.lean_calc(base_pose, lean_amount, 1)
        ankle_lean = self.lean_calc(base_pose, lean_amount, 2)

        # Calculate servo movement speeds
        thigh_speed = lowest_servo_speed *  self._speed_mul[base_pose][base_pose][0]
        knee_speed  = lowest_servo_speed *  self._speed_mul[base_pose][base_pose][1]
        ankle_speed = lowest_servo_speed *  self._speed_mul[base_pose][base_pose][2]

        #print( "DBG: *** Lean2 amount: thigh = %2.3f, knee = %2.3f, ankle = %2.3f" % (thigh_lean, knee_lean, ankle_lean ))
        

        # Set Speeds
        SetSingleServoSpeed(thigh_speed, 'right_leg_thigh_lift_joint')
        SetSingleServoSpeed(thigh_speed, 'left_leg_thigh_lift_joint')
        SetSingleServoSpeed(knee_speed, 'right_leg_knee_bend_joint')
        SetSingleServoSpeed(knee_speed, 'left_leg_knee_bend_joint')
        SetSingleServoSpeed(ankle_speed, 'right_leg_ankle_rotate_joint')
        SetSingleServoSpeed(ankle_speed, 'left_leg_ankle_rotate_joint')


        if direction == 'RIGHT':
            rospy.loginfo("%s: Leaning Right" % (self._calling_name))

            # Right legs down
            pub_right_leg_thigh_lift.publish(thigh_lean)
            pub_right_leg_knee_bend.publish(knee_lean)
            pub_right_leg_ankle_rotate.publish(ankle_lean)

            # Left legs back to neutral
            pub_left_leg_thigh_lift.publish(thigh_neutral)
            pub_left_leg_knee_bend.publish(knee_neutral)
            pub_left_leg_ankle_rotate.publish(ankle_neutral)
            
        elif direction == 'CENTER_LOW':
            rospy.loginfo("%s: Leaning Center Low" % (self._calling_name))
            
            # Right legs down
            pub_right_leg_thigh_lift.publish(thigh_lean)
            pub_right_leg_knee_bend.publish(knee_lean)
            pub_right_leg_ankle_rotate.publish(ankle_lean)

            # Left legs down
            pub_left_leg_thigh_lift.publish(thigh_lean)
            pub_left_leg_knee_bend.publish(knee_lean)
            pub_left_leg_ankle_rotate.publish(ankle_lean)

        elif direction == 'CENTER_HIGH' or direction == 'CENTER':
            rospy.loginfo("%s: Leaning Center High" % (self._calling_name))
            
            # Right legs back to neutral
            pub_right_leg_thigh_lift.publish(thigh_neutral)
            pub_right_leg_knee_bend.publish(knee_neutral)
            pub_right_leg_ankle_rotate.publish(ankle_neutral)

            # Left legs back to neutral
            pub_left_leg_thigh_lift.publish(thigh_neutral)
            pub_left_leg_knee_bend.publish(knee_neutral)
            pub_left_leg_ankle_rotate.publish(ankle_neutral)

        elif direction == 'LEFT':
            rospy.loginfo("%s: Leaning Left" % (self._calling_name))
            
            # Left legs down
            pub_left_leg_thigh_lift.publish(thigh_lean)
            pub_left_leg_knee_bend.publish(knee_lean)
            pub_left_leg_ankle_rotate.publish(ankle_lean)

            # Right legs back to neutral
            pub_right_leg_thigh_lift.publish(thigh_neutral)
            pub_right_leg_knee_bend.publish(knee_neutral)
            pub_right_leg_ankle_rotate.publish(ankle_neutral)

        else:
            rospy.logwarn("%s: ERROR! lean direction [%s] not RIGHT, CENTER_LOW, CENTER_HIGH, or LEFT!" % (self._calling_name, direction))
            return False # Fail  

        # print( "set_pose.lean: *********** DONE **********")
        return True # Success     

    # =================================================================================================
    def lean(self, direction, lean_amount=0.12, lowest_servo_speed=0.3):

        # ?? lean_amount NOT USED! I think this is incorrect


        base_pose = self.get_current_pose()

        # Get servo neutral goal positions
        thigh_neutral     = self._pos_lookup[base_pose][0]
        knee_neutral      = self._pos_lookup[base_pose][1]        
        ankle_neutral     = self._pos_lookup[base_pose][2]
        #neck_neutral      = self._pos_lookup[base_pose][3]
        #head_tilt_neutral = self._pos_lookup[base_pose][4]
        #antenna_neutral   = self._pos_lookup[base_pose][5] 

        # Get servo lean goal positions
        thigh_lean      = self._lean_lookup[base_pose][0]
        knee_lean       = self._lean_lookup[base_pose][1]        
        ankle_lean      = self._lean_lookup[base_pose][2]
        #neck_lean      = self._lean_lookup[base_pose][3]
        #head_tilt_lean = self._lean_lookup[base_pose][4]
        #antenna_lean   = self._lean_lookup[base_pose][5] 



        # DEBUG Get servo lean goal positions
        dbg_thigh_lean = self.lean_calc(base_pose, lean_amount, 0)
        dbg_knee_lean = self.lean_calc(base_pose, lean_amount, 1)
        dbg_ankle_lean = self.lean_calc(base_pose, lean_amount, 2)

        print( "*** Lookup: thigh = %2.3f, knee = %2.3f, ankle = %2.3f" % (thigh_lean, knee_lean, ankle_lean ))
        print( "*** Calc:   thigh = %2.3f, knee = %2.3f, ankle = %2.3f" % (dbg_thigh_lean, dbg_knee_lean, dbg_ankle_lean ))
        

        # Calculate servo movement speeds
        thigh_speed = lowest_servo_speed *  self._speed_mul[base_pose][base_pose][0]
        knee_speed  = lowest_servo_speed *  self._speed_mul[base_pose][base_pose][1]
        ankle_speed = lowest_servo_speed *  self._speed_mul[base_pose][base_pose][2]
        print( " *** set_pose: Goal Speeds from pose %d to pose %d: thigh = %2.3f, knee = %2.3f ankle = %2.3f" % (self._current_pose, base_pose, thigh_speed, knee_speed, ankle_speed))

        # Set Speeds
        SetSingleServoSpeed(thigh_speed, 'right_leg_thigh_lift_joint')
        SetSingleServoSpeed(thigh_speed, 'left_leg_thigh_lift_joint')
        SetSingleServoSpeed(knee_speed, 'right_leg_knee_bend_joint')
        SetSingleServoSpeed(knee_speed, 'left_leg_knee_bend_joint')
        SetSingleServoSpeed(ankle_speed, 'right_leg_ankle_rotate_joint')
        SetSingleServoSpeed(ankle_speed, 'left_leg_ankle_rotate_joint')


        # LEGS
        if direction == 'RIGHT':
            rospy.loginfo("%s: Leaning Right" % (self._calling_name))

            # Right legs down
            pub_right_leg_thigh_lift.publish(thigh_lean)
            pub_right_leg_knee_bend.publish(knee_lean)
            pub_right_leg_ankle_rotate.publish(ankle_lean)

            # Left legs back to neutral
            pub_left_leg_thigh_lift.publish(thigh_neutral)
            pub_left_leg_knee_bend.publish(knee_neutral)
            pub_left_leg_ankle_rotate.publish(ankle_neutral)
            
        elif direction == 'CENTER':
            rospy.loginfo("%s: Leaning Center" % (self._calling_name))
            
            # Right legs back to neutral
            pub_right_leg_thigh_lift.publish(thigh_neutral)
            pub_right_leg_knee_bend.publish(knee_neutral)
            pub_right_leg_ankle_rotate.publish(ankle_neutral)

            # Left legs back to neutral
            pub_left_leg_thigh_lift.publish(thigh_neutral)
            pub_left_leg_knee_bend.publish(knee_neutral)
            pub_left_leg_ankle_rotate.publish(ankle_neutral)

        elif direction == 'LEFT':
            rospy.loginfo("%s: Leaning Left" % (self._calling_name))
            
            # Left legs down
            pub_left_leg_thigh_lift.publish(thigh_lean)
            pub_left_leg_knee_bend.publish(knee_lean)
            pub_left_leg_ankle_rotate.publish(ankle_lean)

            # Right legs back to neutral
            pub_right_leg_thigh_lift.publish(thigh_neutral)
            pub_right_leg_knee_bend.publish(knee_neutral)
            pub_right_leg_ankle_rotate.publish(ankle_neutral)

        else:
            rospy.logwarn("%s: ERROR! lean direction [%s] not RIGHT, CENTER, or LEFT!" % (self._calling_name, direction))
            return False # Fail  

        print( "set_pose.lean: *********** DONE **********")
        return True # Success     


    # =================================================================================================
    def move(self, goal_pose, lowest_servo_speed=0.3, head_center=True, table_mode=False):

        if self._current_pose < 0:
            # initial pose not set yet.
            self.get_initial_pose()            

        # This is used to tilt head up when on the ground, or look straight ahead when on a table-top
        if goal_pose == 0:
            head_tilt_offset = 0
        elif table_mode:
            head_tilt_offset = TABLE_MODE_HEAD_TILT_OFFSET
        else:
            head_tilt_offset = FLOOR_MODE_HEAD_TILT_OFFSET


        print( "DBG: set_pose: Starting Pose: %d  Goal Pose: %d" % (self._current_pose, goal_pose))

        # get current position of servos.  NOTE! This is currently only used to print debug info!
        (position, velocity, effort) = self.call_return_joint_states( \
            ['right_leg_thigh_lift_joint', 'right_leg_knee_bend_joint', 'right_leg_ankle_rotate_joint'])
        right_leg_thigh_lift_pos = position[0]
        right_leg_knee_bend_pos = position[1]
        right_leg_ankle_rotate_pos = position[2]
        # ' '.join(['%2.3f'%x for x in list])
        print( " *** set_pose: Current Positions: thigh = %2.3f, knee = %2.3f, ankle = %2.3f" % (right_leg_thigh_lift_pos, right_leg_knee_bend_pos, right_leg_ankle_rotate_pos))

        print("DBG: set_pose: Setting servo torque to 1.0 for all joints")         
        SetServoTorque(1.0, all_servo_joints)
        print("DBG: set_pose: Setting servo torque to 1.0 for all joints -- DONE")         

        # Get servo goal positions
        thigh_goal     = self._pos_lookup[goal_pose][0]
        knee_goal      = self._pos_lookup[goal_pose][1]        
        ankle_goal     = self._pos_lookup[goal_pose][2]

        neck_goal      = self._pos_lookup[goal_pose][3] 
        head_tilt_goal = self._pos_lookup[goal_pose][4] + head_tilt_offset
        antenna_goal   = self._pos_lookup[goal_pose][5] 


        # For testing robot walking on it's head        
        DEBUG_HEAD_DOWN = False   # TODO-DAVE TURN OFF THIS DEBUG HACK!!! 
        DEBUG_HEAD_WALK = False
           
        if DEBUG_HEAD_DOWN:      
            neck_goal      = -1.50
            head_tilt_goal = -1.47
            antenna_goal   =  1.50  
              
        elif DEBUG_HEAD_WALK and (goal_pose != 0):
            neck_goal      = -0.80
            head_tilt_goal = -0.70
            antenna_goal   =  1.50    


        print( " *** set_pose: Goal Positions: thigh = %2.3f, knee = %2.3f, ankle = %2.3f, neck = %2.3f, head tilt = %2.3f, antennas = %2.3f" % (thigh_goal, knee_goal, ankle_goal, neck_goal, head_tilt_goal, antenna_goal))


        # Calculate servo movement speeds
        thigh_speed = lowest_servo_speed *  self._speed_mul[self._current_pose][goal_pose][0]
        knee_speed  = lowest_servo_speed *  self._speed_mul[self._current_pose][goal_pose][1]
        ankle_speed = lowest_servo_speed *  self._speed_mul[self._current_pose][goal_pose][2]
        print( " *** set_pose: Goal Speeds from pose %d to pose %d: thigh = %2.3f, knee = %2.3f ankle = %2.3f" % (self._current_pose, goal_pose, thigh_speed, knee_speed, ankle_speed))

        # Set servos speed 
        # TODO THIS MESSES UP BEHAVIORS!  Need to save speed and restore when done. See KLUDGE in Face Tracker.
        
        head_speed = lowest_servo_speed * 2.5 # Set all head servos to base speed
        if self._current_pose == 0 or goal_pose == 0:
            # make sure servo is moving fast enough to engage/disengage magnets
            print( " *** set_pose: To or from Pose 0! Increasing head speed!")
            if head_speed < MAGNET_DISENGAGE_SPEED:
                head_speed = MAGNET_DISENGAGE_SPEED

        SetServoSpeed(head_speed, head_joints) 
        SetSingleServoSpeed(0.9, 'right_antenna_joint') # longer distance to travel
        SetSingleServoSpeed(0.9, 'left_antenna_joint') # longer distance to travel
        

        SetSingleServoSpeed(thigh_speed, 'right_leg_thigh_lift_joint')
        SetSingleServoSpeed(thigh_speed, 'left_leg_thigh_lift_joint')
        
        SetSingleServoSpeed(knee_speed, 'right_leg_knee_bend_joint')
        SetSingleServoSpeed(knee_speed, 'left_leg_knee_bend_joint')
        
        SetSingleServoSpeed(ankle_speed, 'right_leg_ankle_rotate_joint')
        SetSingleServoSpeed(ankle_speed, 'left_leg_ankle_rotate_joint')

        # Base speed
        # SetSingleServoSpeed(lowest_servo_speed, 'left_leg_ankle_rotate_joint')

            
        #time.sleep(1) # is this needed?

        # Move head to good position
        if head_center:
            pub_head_pan.publish(0.0)
            pub_head_sidetilt.publish(0.0)
            rospy.sleep(1) # is this needed to allow head to turn forward before lowering the neck?

        # move the head
        pub_neck_raise.publish(neck_goal)
        pub_head_tilt.publish(head_tilt_goal)
        pub_right_antenna.publish(antenna_goal)
        pub_left_antenna.publish(antenna_goal)

        # move the legs
        pub_right_leg_thigh_lift.publish(thigh_goal)
        pub_left_leg_thigh_lift.publish(thigh_goal)

        pub_right_leg_knee_bend.publish(knee_goal)
        pub_left_leg_knee_bend.publish(knee_goal)

        pub_right_leg_ankle_rotate.publish(ankle_goal)
        pub_left_leg_ankle_rotate.publish(ankle_goal)

        # update the current pose
        self._current_pose = goal_pose
        
        # See if any servos need to be repositioned after moving into position
        # (this keeps servos from overheating)       
        if goal_pose == 0 or goal_pose == 1:
            print("Sleeping to allow servo move before final servo REST position")
            rospy.sleep(2.0)
            
            print("Resetting Servos to REST positions (less load)")
            thigh_goal     = self._pos_lookup[REST_POSE][0]
            knee_goal      = self._pos_lookup[REST_POSE][1]        
            ankle_goal     = self._pos_lookup[REST_POSE][2]
            neck_goal      = self._pos_lookup[REST_POSE][3]
            
            pub_right_leg_thigh_lift.publish(thigh_goal)
            pub_left_leg_thigh_lift.publish(thigh_goal)
            pub_right_leg_knee_bend.publish(knee_goal)
            pub_left_leg_knee_bend.publish(knee_goal)
            pub_right_leg_ankle_rotate.publish(ankle_goal)
            pub_left_leg_ankle_rotate.publish(ankle_goal)
            
            if goal_pose == 0:
                pub_neck_raise.publish(neck_goal)    
        
        print("set_pose: New Pose = ", self._current_pose)
        print( "set_pose: *********** DONE **********")

        
if __name__=='__main__':

    total = len(sys.argv)
    cmdargs = str(sys.argv)
    robotpose = RobotPose("set_pose")
    
    rospy.init_node('set_pose', anonymous=True)
    if total > 2:
        goal_pose = int(sys.argv[1])
        lowest_servo_speed = float(sys.argv[2])

        if goal_pose == 10:
            robotpose.pose_down(lowest_servo_speed)
        elif goal_pose == 11:
            robotpose.pose_up(lowest_servo_speed)
        else:
            robotpose.move(goal_pose, lowest_servo_speed)

    else:
        print( 'USAGE: set_pose.py <pose_num (0-5)> <servo_speed (0.2 - 4.0)> ')
        #sys.exit()
        
        
        


