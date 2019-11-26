#! /usr/bin/env python

import rospy
import numpy as np
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from robot_control import RobotBaseMove


class BlockDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.img_cb)
        self.block_pub = rospy.Publisher('detect/block', Bool, queue_size=1)
        self.robot_controller = RobotBaseMove()

    def img_cb(self, msg):
        origin_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        lower_red = np.array([0, 0, 90])
        upper_red = np.array([5, 5, 110])
        gray_img = cv2.inRange(origin_image, lower_red, upper_red)

        h, w = gray_img.shape
        block_bar_mask = gray_img
        block_bar_mask[0:180, 0:w] = 0
        block_bar_mask[240:h, 0:w] = 0

        block_bar_mask, contours, hierarchy = cv2.findContours(block_bar_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # for i in range(len(contours)):
        # print (len(contours))
        if len(contours) == 0:
            self.block_pub.publish(True)
            self.robot_controller.go_forward()
            self.robot_controller.set_velocity(1)
        else:
            self.block_pub.publish(False)
            self.robot_controller.set_velocity(0)
        # self.robot_controller.go_forward()


if __name__ == '__main__':
    rospy.init_node('test_node')
    line_finder = BlockDetector()
    rospy.spin()

