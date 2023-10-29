import math
import copy


def box_movement(initial_pose, lenght, height):
    assert height < (initial_pose.position.z - 0.1)

    waypoints = []
    #center of upper face of box
    cx = initial_pose.position.x
    cy = initial_pose.position.y

    pose = initial_pose

    #get to first corner
    pose.position.x -= (lenght/2)
    pose.position.y -= (lenght/2)

    waypoints.append(copy.deepcopy(pose))

    #get down, then up again
    pose.position.z -= height
    waypoints.append(copy.deepcopy(pose))
    pose.position.z += height
    waypoints.append(copy.deepcopy(pose))

    #get to second corner
    pose.position.y += lenght

    waypoints.append(copy.deepcopy(pose))

    #get down, then up again
    pose.position.z -= height
    waypoints.append(copy.deepcopy(pose))
    pose.position.z += height
    waypoints.append(copy.deepcopy(pose))

    #get to third corner
    pose.position.x += lenght

    waypoints.append(copy.deepcopy(pose))

    #get down, then up again
    pose.position.z -= height
    waypoints.append(copy.deepcopy(pose))
    pose.position.z += height
    waypoints.append(copy.deepcopy(pose))

    #get to last corner
    pose.position.y -= lenght

    waypoints.append(copy.deepcopy(pose))

    #get down, then up again
    pose.position.z -= height
    waypoints.append(copy.deepcopy(pose))
    pose.position.z += height
    waypoints.append(copy.deepcopy(pose))

    #return initial pos
    pose.position.x -= (lenght/2)
    pose.position.y += (lenght/2)
    waypoints.append(copy.deepcopy(pose))

    return waypoints