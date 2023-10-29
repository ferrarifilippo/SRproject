#!/usr/bin/env python3

import rospy
import numpy as np
from visual_kinematics.RobotSerial import *
#from trajectory_msgs import msg
from threading import Thread
from sensor_msgs import msg


#examples of positions (one number per joint)
pos_rest = [0.0, -1.5, 1.0, 0.0, 0.0, 0.0]

#thread for constantly publishing a message to set the robot in position
class Thread_msgs (Thread):
   def __init__(self, message, pub, rate):
      Thread.__init__(self)
      self.message = message
      self.pub = pub
      self.rate = rate
      self.stop = False

   def run(self):
      print("Thread avviato")

      while not rospy.is_shutdown():
         self.pub.publish(self.message)
         self.rate.sleep()
         if(self.stop == True):
            break

      print("Thread terminato")



#function to get the joint position from end effector position
def inverse_kinem(pos, robot):
   tmp = robot.axis_values

   pos = pos.split()
   if(len(pos) != 6):
      print("errore!")
      robot.forward(tmp)
      return -tmp
   
   for i in range(len(pos)):
      pos[i] = float(pos[i])

   pos_end = np.array([[pos[0]], [pos[1]], [pos[2]]])
   rot_end = np.array([pos[5], -pos[4], pos[3]+pi/2])
   end = Frame.from_euler_3(rot_end, pos_end)
   robot.inverse(end)

   if(robot.is_reachable_inverse):
      print("raggiungibile")
      return -robot.axis_values
   else:
      print("non raggiungibile")
      robot.forward(tmp)
      return -tmp



#new ros node, for publishing positions of joints
rospy.init_node('move_cougarbot')
pub = rospy.Publisher('ur5/joint_states', msg.JointTrajectory, queue_size=1)
rate = rospy.Rate(50)
'''
#new robot for inverse kinematics
dh = np.array([
                      [0.50, 0.0, pi/2, 0.0],
                      [0.0, 0.35, 0.0, 0.0],
                      [0.0, 0.4, 0.0, 0.0],
                      [0.0, 0.008, 0.0, 0.0]
                      ])
robot = RobotSerial(dh)
robot2 = RobotSerial(dh)
'''

#new message for positions
message = msg.JointState()
jtp = msg.JointTrajectoryPoint()
jtp.positions = pos_rest 
jtp.time_from_start = rospy.Duration.from_sec(1.0)

message.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",]
message.points.append(jtp)


#cycle to get the input position and move the robot

while True:
    #start thread for publishing the message
    th = Thread_msgs(message, pub, rate)
    th.start()

    #get new positions for the joint (or cartesian end effector) position
    pos = input()
    th.stop=True
    #create the new message to publish, (using inverse kinematics)
    '''
    axis = inverse_kinem(pos)

    print(axis)
    axis[0] = (-axis[0])
    f = robot2.forward(axis)
    print(f.euler_3)'''

    #add the values for joint movement
    message.points.clear()
    jtp.positions = pos
    message.points.append(jtp)