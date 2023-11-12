import math
from math import atan2
import copy
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



#movement around a room, of 45 degrees each time 
def ambient_movement(initial_pose, forward_reach, up_down_reach):
    #create list of waypoints, and return them
    waypoints = []
    steps = [0, 45, 90, 135, 180, 225, 270, 315, 360]
    
    #center is robot
    cx = 0
    cy = 0
    radius = initial_pose.position.x

    pose = initial_pose
    for theta in steps:
        pin_point = (np.cos(theta*math.pi/180), np.sin(theta*math.pi/180))
        pose.position.y = cy + radius*np.sin(theta*math.pi/180)
        pose.position.x = cx + radius*np.cos(theta*math.pi/180)
        roll, pitch, yaw = 0.0,0.0, get_yaw(pin_point, pose)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = get_quaternion_from_euler(roll, pitch, yaw)

        waypoints.append(copy.deepcopy(pose))
        pose.position.x += forward_reach * np.cos(theta*math.pi/180)
        pose.position.y += forward_reach * np.sin(theta*math.pi/180)
        waypoints.append(copy.deepcopy(pose))
        pose.position.z -= up_down_reach
        waypoints.append(copy.deepcopy(pose))
        pose.position.z += up_down_reach
        waypoints.append(copy.deepcopy(pose))
        pose.position.x -= forward_reach * np.cos(theta*math.pi/180)
        pose.position.y -= forward_reach * np.sin(theta*math.pi/180)
        waypoints.append(copy.deepcopy(pose))


    #roll, pitch, yaw = 0.0,0.0, get_yaw(pin_point, initial_pose)
    #initial_pose.orientation.x, initial_pose.orientation.y, initial_pose.orientation.z, initial_pose.orientation.w = get_quaternion_from_euler(roll, pitch, yaw)
    #starting positions
    #cx = initial_pose.position.x - radius
    #cy = initial_pose.position.y

    return waypoints