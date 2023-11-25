#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

from math import pi, tau, dist, fabs, cos, sin, atan2
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from trajectories.circular_move import circular_movement
from trajectories.box_move import box_movement
from trajectories.bell_move import bell_movement
from trajectories.ambient_move import ambient_movement
from trajectories.rotate_move import rotation_movement



def get_quaternion_from_euler(roll, pitch, yaw):
    #Convert an Euler angle to a quaternion.
    qx = np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) - np.cos(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  
    return qx, qy, qz, qw



#get current orientation to point in the direction (to check if it works)
def get_yaw(pin_point, robot_pose):
    x1, y1 = robot_pose.position.x, robot_pose.position.y
    x2, y2 = pin_point[0], pin_point[1]

    return atan2(y2-y1, x2-x1)



def initial_position_move(robot_move):
    #initial position for robot (to watch from above the object)
    joint_goal = [0.0, -1.5472472410455107, 1.5384722584941635, 0.0, 1.5708548380760137, 0.0]

    #command to initiate the movement (joint or end effect pose)
    robot_move.go(joint_goal, wait=True)

    #to stop movement
    robot_move.stop()
    return 


def set_pose(robot_move, x,y,z, ox, oy, oz, ow):
    pose = robot_move.get_current_pose().pose
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = ox, oy, oz, ow
    robot_move.set_pose_target(pose)
    robot_move.go(wait=True)
    robot_move.stop()


def get_depth_image():
    pass


#initialize robot connection to moveit
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
robot = moveit_commander.RobotCommander()

#group name for movements on joint or cartesian
group_name = "rotot_total"
robot_move = moveit_commander.MoveGroupCommander(group_name)


#start position of robot
initial_position_move(robot_move)

#TODO: spawn object in Rviz and Gazebo, set its central point (where its vetical axis is)
pin_point = (0.5, 0.0)



#movement to check three points: forward, left and right of object
#first position
set_pose(robot_move, 0.30,0.0,0.20, 0.0,0.0,0.0,1.0)

#get picture of object
get_depth_image()


#second position
set_pose(robot_move, 0.50, 0.40, 0.20, 0.0,0.0,0.0,1.0)


#set the orientation (watch left, moving the wrist_2)
joints = robot_move.get_current_joint_values()
joints[4] += 1.57
if(joints[4] > 3.14):
    joints[4] = -6.28 + joints[4]

robot_move.go(joints, wait=True)
robot_move.stop()


#take the picture
get_depth_image()

#reset the robot to initial position
initial_position_move(robot_move)


#third position
set_pose(robot_move, 0.5,-0.4 ,0.2, 0.0,0.0,0.0,1.0)

#set the orientation (watch right)
joints = robot_move.get_current_joint_values()
tmp = joints[4]
joints[4] -= 1.57
if(joints[4] < -3.14):
    joints[4] = 6.28 + joints[4]

robot_move.go(joints, wait=True)
robot_move.stop()

#get third picture
get_depth_image()