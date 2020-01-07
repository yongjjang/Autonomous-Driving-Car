#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from detect_behavior import DetectBehavior
from robot_drive_controller import RobotDriveController


class LaneTracer:
    """
    차선을 인식하여 차선의 중알을 향하여 주행을 하는 책임을 지는 클래스
    """
    def __init__(self, image_topic_name):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(image_topic_name, Image, self.image_callback)
        self.image_pub = rospy.Publisher(image_topic_name + "/trace_image", Image, queue_size=1)
        # self.detect_behavior = DetectBehavior(StopLineDetectBehavior)
        self.cx = 0
        self.err = 0

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        h, s, v = cv2.split(hsv_image)
        v = cv2.inRange(v, 210, 220)
        # v[0:300, 0:640] = 0
        height, width, _ = hsv_image.shape
        hsv_image[height/2:0, 0:width] = 0

        M = cv2.moments(v)  # 도형의 무게중심을 계산
        if M['m00'] > 0:
            self.cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (self.cx, cy), 20, (0, 255, 0), -1)
            self.cx = self.cx - 320

        image = self.bridge.cv2_to_imgmsg(image)
        self.image_pub.publish(image)


if __name__ == '__main__':
    rospy.init_node('Lane_Tracer')
    left_line = LaneTracer('my_left_camera/rgb/image_raw')
    right_line = LaneTracer('my_right_camera/rgb/image_raw')
    drive_controller = RobotDriveController()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        cx = (left_line.cx + right_line.cx)/2
        err = -float(cx)/80
        print err
        if abs(err) > 0.14:
            drive_controller.set_velocity(0.4)
            drive_controller.set_angular(err)
        elif abs(err) < 0.14:
            drive_controller.set_velocity(1)
            drive_controller.set_angular(err)
        drive_controller.drive()
        rate.sleep()

