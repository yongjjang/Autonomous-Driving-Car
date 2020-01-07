#! /usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from base_detector import BaseDetector
import os


class DetectStopSign(BaseDetector):
    """
    정지 표지판을 탐지하고, 탐지될 경우 메시지를 발행하는 클래스

    """
    def __init__(self):
        super(DetectStopSign, self).__init__()
        self.sift = cv2.xfeatures2d.SIFT_create()
        self.stop_sign_image = cv2.imread(os.path.abspath('./stop_sign.png'))  # 어느 환경에서든 사진을 찾을  수 있음.
        self.image_sub = rospy.Subscriber('my_left_camera/rgb/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('stop_sign_detect', Image, queue_size=1)
        self.detect_stop_sign = rospy.Publisher('detect/is_stop_sign', Bool, queue_size=1)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        kp1, des1 = self.sift.detectAndCompute(image, None)
        kp2, des2 = self.sift.detectAndCompute(self.stop_sign_image, None)

        bf = cv2.BFMatcher()
        matches = bf.match(des1, des2)
        sorted_matches = sorted(matches, key=lambda x: x.distance)
        stop_sign_image = cv2.drawMatches(image, kp1, self.stop_sign_image, kp2, sorted_matches[:80], None, flags=2)
        stop_sign_image = self.bridge.cv2_to_imgmsg(stop_sign_image, encoding="passthrough")
        self.image_pub.publish(stop_sign_image)

        if len(matches) < 100:
            return
        elif len(matches) >= 100 and sorted_matches[0] > 400:
            # print sorted_matches[80].distance
            self.detect_stop_sign.publish(True)
            # self.current_focus_pub.publish(stop_sign_image)



if __name__ == "__main__":
    rospy.init_node('Detect_Stop_Sign')
    test = DetectStopSign()
    while not rospy.is_shutdown():
        self.rate.sleep()