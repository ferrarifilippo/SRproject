from math import pi, sin, cos
import copy

#movement across num circles going down 
def circular_movement(initial_pose, radius, num_circles):
    #create list of waypoints, and return them
    waypoints = []

    final_pose = copy.deepcopy(initial_pose)

    #starting positions
    cx = initial_pose.position.x - radius
    cy = initial_pose.position.y

    cz = initial_pose.position.z
    #define the z down movement, in order to stay 0.1 m up
    cz = (cz - 0.1) / num_circles

    for _ in range(num_circles):
        #create circle on x,y direction 
        for theta in range(1,360):
            initial_pose.position.y = cy + radius*sin(theta*pi/180)
            initial_pose.position.x = cx + radius*cos(theta*pi/180)
            waypoints.append(copy.deepcopy(initial_pose))

        initial_pose.position.z -= cz
        waypoints.append(copy.deepcopy(initial_pose))

    #return to first point 
    #waypoints.append(final_pose)

    return waypoints