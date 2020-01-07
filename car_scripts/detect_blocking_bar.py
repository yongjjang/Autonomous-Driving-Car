#! /usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
import cv2
from base_detector import BaseDetector
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from robot_drive_controller import RobotDriveController


class DetectBlockingBar(BaseDetector):
    """
    차단바를 탐지하고, 탐지될 경우 메시지를 발행하는 클래스
    """
    def __init__(self):
        super(DetectBlockingBar, self).__init__()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.image_pub = rospy.Publisher('detect/blocking_bar', Image, queue_size=1)
        self.detect_block_pub = rospy.Publisher('detect/is_block', Bool, queue_size=1)
        self.len_contour = 0
        self.drive_controller = RobotDriveController()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        lower_red = np.array([0, 0, 90])
        upper_red = np.array([5, 5, 110])
        gray_img = cv2.inRange(image, lower_red, upper_red)
        h, w = gray_img.shape
        block_bar_mask = gray_img

        block_bar_mask[h/2:h, 0:w] = 0
        # block_bar_mask[240:h, 0:w] = 0

        block_bar_mask, contours, hierarchy = cv2.findContours(block_bar_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        block_bar_mask = self.bridge.cv2_to_imgmsg(block_bar_mask, "passthrough")
        self.len_contour = len(contours)
        if contours:
            self.image_pub.publish(block_bar_mask)
            if len(contours) >= 3:

                self.detect_block_pub.publish(True)
            else:
                self.detect_block_pub.publish(False)


if __name__ == '__main__':
    rospy.init_node('test_node')
    detecter = DetectBlockingBar()
    drive_controller = RobotDriveController
    while not rospy.is_shutdown():
        # print detecter.len_contour

        if detecter.len_contour < 2:
            detecter.drive_controller.set_velocity(1)
            detecter.drive_controller.drive()
        else:
            detecter.drive_controller.set_velocity(0)
            detecter.drive_controller.drive()
        detecter.rate.sleep()


