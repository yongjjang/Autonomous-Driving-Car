#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class LineFollower:
    def __init__(self, image_topic):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(image_topic, Image, self.left_image_callback)
        self.image_pub = rospy.Publisher(image_topic + "circle", Image, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        image = self.image_process(msg)
        self.image_pub.publish(image)

    def image_process(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


        _, _, v = cv2.split(hsv)

        mask = cv2.inRange(, lower_yellow, upper_yellow)

        h, w, d = image.shape
        search_top = 3 * h / 4
        search_bot = search_top + 20

        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        M = cv2.moment(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 20, (120, 120, 120), -1)
            # err = cx - w / 2
            # self.twist.linear.x = 0.5
            # self.twist.angular.z = -float(err)/100
            # self.cmd_vel_pub.publish(self.twist)
        circle_img = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        return circle_img

if __name__ == "__main__":
    follower = LineFollower()
    rospy.spin()



