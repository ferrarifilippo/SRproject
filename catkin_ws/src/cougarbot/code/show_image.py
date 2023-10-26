#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def read_image(message):
    image = cvb.imgmsg_to_cv2(message, desired_encoding="passthrough")
    cv2.imshow("prova", image)
    cv2.waitKey(30)


cvb = CvBridge()
rospy.init_node("read_image")
sub = rospy.Subscriber('/camera/color/image_raw', Image, read_image)
rospy.spin()