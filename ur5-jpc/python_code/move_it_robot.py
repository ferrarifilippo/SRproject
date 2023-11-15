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
from trajectories.bell_move import bell_movement
from trajectories.ambient_move import ambient_movement
import gazebo_msgs.msg 
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_msgs.srv import SpawnModel, DeleteModel
from xml.etree import ElementTree as ET
from std_msgs.msg import Header

import save_depth


def initial_position_move(robot_move):
    #initial position for robot (to watch from above the object)
    joint_goal = [0, -90*pi/180, 90*pi/180, -90*pi/180, -90*pi/180, 0]

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
group_name = "rotot_total"
robot_move = moveit_commander.MoveGroupCommander(group_name)

#start position of robot
initial_position_move(robot_move)

#start orientation of robot (looking forward)
wpose = robot_move.get_current_pose().pose
wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w = 0.0, 0.0, 0.0, 1.0
robot_move.set_pose_target(wpose)
robot_move.go(wait=True)
robot_move.stop()
# Nome del modello
model_name = "beer"

# Percorso del file SDF del modello fountain
sdf_path = "/home/filippo/.gazebo/models/beer/model.sdf"

# Chiamata al servizio di Gazebo per eliminare il modello se esiste già
rospy.wait_for_service('/gazebo/delete_model')
try:
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    delete_model(model_name)
except rospy.ServiceException as e:
    rospy.loginfo(f"Model '{model_name}' does not exist yet or could not be deleted: {e}")

# Chiamata al servizio di Gazebo per spawnare la fontana
rospy.wait_for_service('/gazebo/spawn_sdf_model')
try:
    spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Leggi il file SDF dopo le modifiche
    with open(sdf_path, "r") as f:
        sdf = f.read()

    # Effettua lo spawn del modello con le dimensioni modificate
    spawn_sdf(model_name, sdf, "/", beer_pose.pose, "world")

except rospy.ServiceException as e:
    rospy.logerr("Service call failed: %s" % e)



#spawn object on scene below robot position, first create scene
scene = moveit_commander.PlanningSceneInterface()

rospy.sleep(2)

p = geometry_msgs.msg.PoseStamped()
p.header.frame_id = robot.get_planning_frame()

#where to spawn the object
p.pose.position.x = 0.75
p.pose.position.y = 0.15
p.pose.position.z = 0.15
#spawn and dimensions of object
scene.add_box("example box", p, (0.12, 0.12, 0.26))



print(robot_move.get_current_pose().pose.position.x, 
      robot_move.get_current_pose().pose.position.y, 
      robot_move.get_current_pose().pose.position.z)

print(robot_move.get_current_rpy())


#perform first circular movement
wpose = robot_move.get_current_pose().pose

wpose.position.x, wpose.position.y = 0.15, 0.15
robot_move.set_pose_target(wpose)
robot_move.go(wait=True)
robot_move.stop()


#radius and num circles
r = 0.2
num_circles = 3
waypoints = circular_movement(wpose, r, num_circles)

plan, _ = robot_move.compute_cartesian_path(
    waypoints, 0.005, 0.0  # waypoints to follow  # eef_step
)  # jump_threshold


robot_move.execute(plan, wait=False)

save_depth.main()

'''
#perform box movement
lenght = 0.1
height = 0.1
waypoints = box_movement(wpose, lenght, height)

plan, _ = robot_move.compute_cartesian_path(
    waypoints, 0.005, 0.0  # waypoints to follow  # eef_step
)  # jump_threshold

robot_move.execute(plan, wait=True)


#radius of bell movement
r = 0.1
waypoints = bell_movement(robot_move.get_current_pose().pose, r, r)

plan, _ = robot_move.compute_cartesian_path(
    waypoints, 0.005, 0.0  # waypoints to follow  # eef_step
)  # jump_threshold

robot_move.execute(plan, wait=True)
'''

'''
#perform ambient scanning
pose = robot_move.get_current_pose().pose
up_down = 0.2
forward = 0.1
waypoints = ambient_movement(pose, forward_reach=forward, up_down_reach=up_down)

plan, _ = robot_move.compute_cartesian_path(
    waypoints, 0.002, 0.0  # waypoints to follow  # eef_step
)  # jump_threshold

robot_move.execute(plan, wait=True)

print(robot_move.get_current_pose().pose.position.x, 
      robot_move.get_current_pose().pose.position.y, 
      robot_move.get_current_pose().pose.position.z)
    
'''