#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos, sin
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from trajectories.circular_move import circular_movement
from trajectories.box_move import box_movement


def initial_position_move(robot_move):
    #initial position for robot (to watch from above the object)
    joint_goal = [0, -86*pi/180, 95*pi/180, -91*pi/180, -91*pi/180, 0]

    #command to initiate the movement (joint or end effect pose)
    robot_move.go(joint_goal, wait=True)

    #to stop movement
    robot_move.stop()




#init node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

#create robot
robot = moveit_commander.RobotCommander()

#group name for movements on joint or cartesian
group_name = "robot_arm"
robot_move = moveit_commander.MoveGroupCommander(group_name)

'''
#end effector for the group
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link)

#all groups specified for rviz
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names())
'''

#start position of robot
initial_position_move(robot_move)

#spawn object on scene below robot position, first create scene
scene = moveit_commander.PlanningSceneInterface()

rospy.sleep(2)

p = geometry_msgs.msg.PoseStamped()
p.header.frame_id = robot.get_planning_frame()

'''
#where to spawn the object
p.pose.position.x = 0.5
p.pose.position.y = 0.1
p.pose.position.z = 0.15
#spawn and dimensions of object
scene.add_box("example box", p, (0.2, 0.2, 0.3))
'''

print(robot_move.get_current_pose().pose.position.x, 
      robot_move.get_current_pose().pose.position.y, 
      robot_move.get_current_pose().pose.position.z)

#mess with orientation of end effector
wpose = robot_move.get_current_pose().pose
wpose.orientation.x = 0.0
wpose.orientation.y = 0.0
wpose.orientation.z = 0.0
robot_move.set_pose_target(wpose)
robot_move.go(wait=True)
robot_move.stop()

'''
#perform first circular movement
wpose = robot_move.get_current_pose().pose

#radius and num circles
r = 0.1
num_circles = 3
waypoints = circular_movement(wpose, r, num_circles)

plan, _ = robot_move.compute_cartesian_path(
    waypoints, 0.005, 0.0  # waypoints to follow  # eef_step
)  # jump_threshold

robot_move.execute(plan, wait=True)

#perform box movement
lenght = 0.2
height = 0.2
waypoints = box_movement(wpose, lenght, height)

plan, _ = robot_move.compute_cartesian_path(
    waypoints, 0.005, 0.0  # waypoints to follow  # eef_step
)  # jump_threshold

robot_move.execute(plan, wait=True)
'''
print(robot_move.get_current_pose().pose.position.x, 
      robot_move.get_current_pose().pose.position.y, 
      robot_move.get_current_pose().pose.position.z)