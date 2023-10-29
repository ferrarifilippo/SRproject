import math
from math import pi, sin, cos
import copy

def bell_movement(initial_pose, radius, radius_z):
    waypoints = []

    #initial position is the upper point of the bell
    #we go down 4 archs and every time we get up
    cx = initial_pose.position.x
    cy = initial_pose.position.y
    cz = initial_pose.position.z - radius
    #4 quarter-circonferences xz, yz, -yz, -xz 
    for theta in range(1,90):
        initial_pose.position.x = cx + radius*sin(theta*pi/180)
        initial_pose.position.z = cz + radius*cos(theta*pi/180)
        waypoints.append(copy.deepcopy(initial_pose))

    for theta in range(90,0, -1):
        initial_pose.position.x = cx + radius*sin(theta*pi/180)
        initial_pose.position.z = cz + radius*cos(theta*pi/180)
        waypoints.append(copy.deepcopy(initial_pose))

    return waypoints