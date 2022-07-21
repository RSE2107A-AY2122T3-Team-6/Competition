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
    (0,335),
    (640,335),
    (640,440)
]


def read_image(image):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
    cv_image_copy = cv_image.copy()

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    white_mask = cv2.inRange(hsv, (89,50,0), (90,255,255))
    white_res = cv2.bitwise_and(cv_image,cv_image,mask=white_mask)

    img = cv2.cvtColor(white_res, cv2.COLOR_BGR2GRAY)
    #blur_img = cv2.GaussianBlur(img, (5,5), 0)
    canny_img = cv2.Canny(img, 50, 150)

    mask = np.zeros_like(img)
    channel_count = cv_image.shape[2]
    match_mask_color = (255,) * channel_count
    vertices = np.array([interested_region], dtype=np.int32)
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_img = cv2.bitwise_and(img,mask)

    contours, hierarchy = cv2.findContours(masked_img.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #print(contours[0])

    if len(contours) > 0:
        blackbox = cv2.minAreaRect(contours[0])
        (x_min, y_min), (w_min, h_min), ang = blackbox


        ang = int(ang)
        box = cv2.boxPoints(blackbox)
        box = np.int0(box)
        cv2.drawContours(cv_image, [box], 0, (0,0,255), 3)  
        cv2.putText(cv_image, str(ang), (10,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    
    midpt_box = ((box[0][0]+box[2][0])/2,(box[0][1]+box[2][1])/2)
    cv_image = cv2.circle(cv_image, midpt_box, 5, (0,0,255), -1)
    cnt_x = 640/2
    cnt_y = 480

    cv2.line(cv_image, (cnt_x, cnt_y), (cnt_x,0), (0,0,255), 3)
    O = midpt_box[0] - cnt_x
    A = midpt_box[1] - cnt_y
    angle_rad = -math.atan2(A,O)
    angle_deg = (angle_rad/math.pi)*180
    print("RAd",angle_rad)
    print("midpoint", midpt_box)

    cv2.line(cv_image, (cnt_x,cnt_y), midpt_box, (0,0,255), 3)
    # m = (midpt_box[1]- 480) / (midpt_box[0] - 640)
    # angle = np.arctan(m)
    cv2.putText(cv_image, str(angle_deg), midpt_box, cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    cv2.drawContours(cv_image, contours, 0, (0,0,255), 3)
    cv2.imshow('LIMO_POV', cv_image)
    cv2.waitKey(3)

    # mask = np.zeros_like(canny_image)
    # channel_count = cv_image.shape[2]
    # match_mask_color = (255,) * channel_count
    # vertices = np.array([interested_region], dtype=np.int32)
    # cv2.fillPoly(mask, vertices, match_mask_color)
    # masked_img = cv2.bitwise_and(canny_image,mask)

    #cv2.imshow('image block', img_blk)

    #lines = cv2.HoughLinesP(masked_img, rho=1, theta=np.pi/180,threshold=20
    #                         ,lines=np.array([]), minLineLength=20, maxLineGap=300)

    #for line in lines:
    #    for x1,y1,x2,y2 in line:
    #        cv2.line(cv_image_copy,(x1,y1),(x2,y2),(10,255,255), thickness=10)

    #plt.imshow(img)
    #plt.show()
    

def main():
    rospy.init_node('limo_pov_node', anonymous=True)
    rospy.Subscriber('/limo/color/image_raw', Image, read_image)
    rospy.spin()


main()