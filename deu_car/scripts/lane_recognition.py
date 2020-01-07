#!/usr/bin/env python
#-*- coding: utf-8 -*-


import rospy
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
import cv2, cv_bridge
import numpy as np


class LaneRecognition:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', 
                                      Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        
        self.is_first_set = False
        self.is_stopline = False
        self.lane_left_found = False
        self.lane_right_found = False
        
        self.lane_left = -1
        self.lane_right = -1
        self.lane_center = -1
        
        self.img_width = None
        self.img_center = 320

        self.twist = Twist()

        self.rate = rospy.Rate(2.0)
        
    def image_callback(self, msg): # 콜백 함수 정의부
        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_mask = np.array([10, 10, 10])
        upper_mask = np.array([10, 20, 170])

        mask = cv2.inRange(hsv, lower_mask, upper_mask)

        h, w, d = image.shape
        self.img_center = w/2

        region_of_interest_top = 3*h/4     #이미지의 3/4지점 (중간보다 아래) 부분을 관심영역으로 설정
        region_of_interest_bot = region_of_interest_top + 20

        mask[0:region_of_interest_top, 0:w] = 0
        mask[region_of_interest_bot:h, 0:w] = 0
        
        for i in range(w/2-1, -1, -1):
            if mask[region_of_interest_top, i] > 0:
                self.lane_left = i
                image = cv2.circle(image, (i, region_of_interest_top), 10, (255, 0, 0), -1)#BLUE
                self.lane_left_found = True
                break

            if i == w/2 -1:
                self.lane_left_found = False

        for i in range(w/2, w):
            if mask[region_of_interest_top, i] > 0:
                self.lane_right = i
                image = cv2.circle(image, (i, region_of_interest_top), 10, (0, 0, 255), -1)#RED
                self.lane_right_found = True
                break

            if i == w/2:
                self.lane_right_found = False

        if self.lane_left_found and self.lane_right_found:
            self.lane_center = (self.lane_right + self.lane_left)/2
            image = cv2.circle(image, (self.lane_center, region_of_interest_top), 10, (0, 255, 0), -1)#GREEN
        else :
            self.door_center = -1

        # BEGIN CONTROL
        err = self.lane_center - w/2
        self.twist.linear.x = 1.0
        self.twist.angular.z = -float(err) / 100
        self.cmd_vel_pub.publish(self.twist)
        # END CONTROL
        
        cv2.imshow("window", image)
        cv2.imshow("mask", mask)
        cv2.waitKey(3)


if __name__ == "__main__":
  rospy.init_node('LaneRecognition')
  lane_recognition = LaneRecognition()
  rospy.spin()
  # END ALL
