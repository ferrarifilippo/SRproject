#!/usr/bin/env python3

import sys
import copy
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos, sin, atan2
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from trajectories.circular_move import circular_movement
from trajectories.box_move import box_movement
import numpy as np


def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
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



#init node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

#create robot
robot = moveit_commander.RobotCommander()

#group name for movements on joint or cartesian
group_name = "rotot_total"
robot_move = moveit_commander.MoveGroupCommander(group_name)

#mess with orientation of end effector


wpose = robot_move.get_current_pose().pose

print(robot_move.get_current_rpy())

#select a point (x, y) where the camera will watch 
pin_point = (0.8, 0.1)

roll, pitch, yaw = 0.0,0.0, get_yaw(pin_point, wpose)

wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w = get_quaternion_from_euler(roll, pitch, yaw)
robot_move.set_pose_target(wpose)
robot_move.go(wait=True)
robot_move.stop()


print(robot_move.get_current_rpy())
print()