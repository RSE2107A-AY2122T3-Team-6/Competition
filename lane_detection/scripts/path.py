#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import matplotlib.pylab as plt
import numpy as np
import math

interested_region = [
    (0,435),
    (0,420),
    (640,420),
    (640,440)
]

class image():
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None

    def read_image(self,image):
        self.cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")

    def print_image(self):
        cv2.imshow('LIMO_POV', self.cv_image)
        cv2.waitKey(3)

def main():
    ss = image()
    rospy.Subscriber('/limo/color/image_raw', Image, ss.read_image)
    ss.print_image()


if __name__ == '__main__':
    rospy.init_node('limo_pov_node', anonymous=True)
    main()
    rospy.spin()