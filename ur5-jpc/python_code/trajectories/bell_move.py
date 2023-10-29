import math
import copy

def bell_movement(initial_pose, radius_xy, radius_z):
    waypoints = []

    #initial position is the upper point of the bell
    #we go down 4 archs and every time we get up
    
