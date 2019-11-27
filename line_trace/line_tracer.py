#! /usr/bin/env python

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from robot_control import RobotBaseMove

class LineTracer:
    def __init__(self, topic_name):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(topic_name, Image, self.image_callback)
        self.image_pub = rospy.Publisher(topic_name + "/binary", Image, queue_size=1)
        self.t = topic_name
        self.cx = 0

    def image_callback(self, msg):
        origin_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(origin_image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_image)

        v = cv2.inRange(v, 210, 220)
        v[0:300, 0:640] = 0
        M = cv2.moments(v)
        if M['m00'] > 0:
            self.cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(origin_image, (self.cx, cy), 20, (0, 255, 0), -1)
            self.cx = self.cx - 320
        # cv2.imshow(self.t, origin_image)
        origin_image = self.bridge.cv2_to_imgmsg(origin_image)
        self.image_pub.publish(origin_image)
        # cv2.waitKey(0)


if __name__ == '__main__':
    rospy.init_node('test')
    left_line = LineTracer('my_left_camera/rgb/image_raw')
    right_line = LineTracer('my_right_camera/rgb/image_raw')
    rate = rospy.Rate(20)
    test = RobotBaseMove()
    while not rospy.is_shutdown():
        cx = (left_line.cx + right_line.cx)/2
        err = -float(cx)/100
        test.set_velocity(0.7)
        test.set_heading(err)
        test.go_forward()
        rate.sleep()
    rospy.spin()
