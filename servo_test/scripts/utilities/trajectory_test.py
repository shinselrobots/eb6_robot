#!/usr/bin/env python3
# WORKS WITH SHELDON!

# TODO!!! Adjust values below before trying with EB!!!

import roslib
import rospy
import actionlib
import time
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal



class Joint:
        def __init__(self, meta_controller_name):
            #meta_controller_name is one of: 'head', 'left_leg', 'right_leg'
            self.name = meta_controller_name           
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')

            if self.name == 'head':
                rospy.loginfo('Moving Head...')
                self.joint_names = ['head_sidetilt_joint','head_tilt_joint','head_pan_joint']

            elif self.name == 'left_leg':
                rospy.loginfo('Moving Left Leg...')
                self.joint_names = [

                    'left_leg_hip_rotate_joint',   
                    'left_leg_thigh_lift_joint',     'left_leg_knee_bend_joint',
                    'left_leg_ankle_rotate_joint'] 

            else:
                rospy.loginfo('Moving Right Leg...')
                self.joint_names = [

                    'right_leg_hip_rotate_joint',   
                    'right_leg_thigh_lift_joint',     'right_leg_knee_bend_joint',
                    'right_leg_ankle_rotate_joint'] 
            
        def move_joint(self, angles, duration):
            goal = FollowJointTrajectoryGoal()                  
            goal.trajectory.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(duration)                   
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)
              

def main():
            
            #print "ERROR! TODO! FIX CODE BEFORE RUNNING FOR EB!!!"
            #error(0)

            move_duration = 10.0  # 2.0 time in seconds for move to complete!
            #head_group = Joint('head')
            #head_group.move_joint([0.25, 0.5, 0.5], move_duration)
            #head_group.move_joint([-0.25, -0.5, -0.5], move_duration*2.0)
            #head_group.move_joint([0.0, 0.0, 0.0], move_duration)

            right_leg_group = Joint('right_leg')
            rospy.loginfo('Moving Position 1')
            # hip lean, hip rotate, thigh, knee, ankle rotate, ankle lean
            right_leg_group.move_joint( [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], move_duration) # stand
            time.sleep(1.0)
            rospy.loginfo('Moving Postion 2 (Home)')
            right_leg_group.move_joint([-0.0, 0.0, -1.489, 2.695, -1.5708, 0.0], move_duration) # sit
            rospy.loginfo('DONE')

            left_leg_group = Joint('left_leg')
            rospy.loginfo('Moving Position 1')
            left_leg_group.move_joint( [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], move_duration) # stand
            time.sleep(1.0)
            rospy.loginfo('Moving Postion 2 (Home)')
            left_leg_group.move_joint([-0.0, 0.0, -1.489, 2.695, -1.5708, 0.0], move_duration) # sit
            rospy.loginfo('DONE')

                        
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()
