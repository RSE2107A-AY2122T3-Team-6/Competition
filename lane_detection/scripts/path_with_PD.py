#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import matplotlib.pylab as plt
import numpy as np
import math

interested_region = [
    (0,440),
    (0,420),
    (640,420),
    (640,440)
]

class speed:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10)

def shutdown(cx_val):
    s = speed()
    cmd = Twist()

    prev_error = 0
    err = cx_val - 640/2
    D = err - prev_error
    
    PID = err + D 

    cmd.linear.x = 0.5
    cmd.angular.z = -float(PID)/100
    print(PID)
    s.pub.publish(cmd)
        
        
def read_image(image):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    white_mask = cv2.inRange(hsv, (81,50,0), (90,255,255))
    white_res = cv2.bitwise_and(cv_image,cv_image,mask=white_mask)

    img = cv2.cvtColor(white_res, cv2.COLOR_BGR2GRAY)

    mask = np.zeros_like(img)
    channel_count = cv_image.shape[2]
    match_mask_color = (255,) * channel_count
    vertices = np.array([interested_region], dtype=np.int32)
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_img = cv2.bitwise_and(img,mask)

    contours, hierarchy = cv2.findContours(masked_img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cv_image, (cx,cy), 20, (0,0,255),-1)
            shutdown(cx)

    cv2.imshow('LIMO_POV', cv_image)
    cv2.waitKey(3)
   
    

def main():
    rospy.init_node('path', anonymous=False)
    rospy.Subscriber('/limo/color/image_raw', Image, read_image)
    rospy.spin()


main()