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
#from trajectories.rotate_move import rotation_movement

import gazebo_msgs.msg 
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_msgs.srv import SpawnModel, DeleteModel
from xml.etree import ElementTree as ET
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2



def initial_position_move(robot_move):
    #initial position for robot (to watch from above the object)
    joint_goal = [0.0, -1.5472472410455107, 1.5384722584941635, 0.0, 1.5708548380760137, 0.0]

    #command to initiate the movement (joint or end effect pose)
    robot_move.go(joint_goal, wait=True)

    #to stop movement
    robot_move.stop()
    return 


def set_pose(robot_move, x,y,z, ox=0.0, oy=0.0, oz=0.0, ow=1.0, set_ori=False):
    pose = robot_move.get_current_pose().pose
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    if(set_ori):
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = ox, oy, oz, ow
    robot_move.set_pose_target(pose)
    robot_move.go(wait=True)
    robot_move.stop()
    return pose


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
#spawn object on scene below robot position, first create scene
scene = moveit_commander.PlanningSceneInterface()

rospy.sleep(2)

p = geometry_msgs.msg.PoseStamped()
p.header.frame_id = robot.get_planning_frame()

#where to spawn the object
p.pose.position.x = 0.5
p.pose.position.y = 0.0
p.pose.position.z = 0.11
#spawn and dimensions of object
(0.316618, 0.330564, 0.089582)
scene.add_box("example box", p, (0.23, 0.23, 0.22))


#movement to check three points: forward, left and right of object
#first position
set_pose(robot_move, 0.30, 0.0, 0.25)

#get picture of object


#second position
set_pose(robot_move, 0.30, 0.40, 0.25)


#set the orientation (watch left, moving the wrist_2)
joints = robot_move.get_current_joint_values()
joints[4] += 0.7
if(joints[4] > 3.14):
    joints[4] = -6.28 + joints[4]

robot_move.go(joints, wait=True)
robot_move.stop()

#third position
set_pose(robot_move, 0.70, 0.40, 0.25, set_ori=True)

#set the orientation (watch left, moving the wrist_2)
joints = robot_move.get_current_joint_values()
joints[4] += 1.5
if(joints[4] > 3.14):
    joints[4] = -6.28 + joints[4]

robot_move.go(joints, wait=True)
robot_move.stop()


#take the picture


#reset the robot to initial position
#set_pose(robot_move, 0.3, 0.0, 0.25, set_ori=True)
initial_position_move(robot_move)

#third position
set_pose(robot_move, 0.3,-0.4 ,0.25, set_ori=True)

#set the orientation (watch right)
joints = robot_move.get_current_joint_values()
tmp = joints[4]
joints[4] -= 1.57/2
if(joints[4] < -3.14):
    joints[4] = 6.28 + joints[4]

robot_move.go(joints, wait=True)
robot_move.stop()


#get third picture




#third position
set_pose(robot_move, 0.65,-0.4 ,0.25, set_ori=True)

#set the orientation (watch right)
joints = robot_move.get_current_joint_values()
tmp = joints[4]
joints[4] -= 1.57
if(joints[4] < -3.14):
    joints[4] = 6.28 + joints[4]

robot_move.go(joints, wait=True)
robot_move.stop()

print('END')