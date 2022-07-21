#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import matplotlib.pylab as plt
import numpy as np
import math

cv_image = None

def read_image(image):
    bridge = CvBridge()
    global cv_image 
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")


def print_image():
    cv2.imshow('LIMO_POV', cv_image)
    cv2.waitKey(3)

def main():
    print_image()

if __name__ == '__main__':
    rospy.init_node('limo_pov_node', anonymous=True)
    rospy.Subscriber('/limo/color/image_raw', Image, read_image)
    main()
    rospy.spin()