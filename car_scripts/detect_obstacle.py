#!/usr/bin/env python
# coding=utf-8

import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from robot_drive_controller import RobotDriveController


class DetectObstacle:
    """
    장애물을 탐지하고, 장애물이 탐지되었을 경우 메세지를 발행하는 클래스
    """
    def __init__(self):
        self.range_ahead = 0
        self.range_right = 0
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.detect_obstacle_pub = rospy.Publisher('detect/is_obstacle', Bool, queue_size=1)
        self.drive_controller = RobotDriveController()
        self.rate = rospy.Rate(20)

    def scan_callback(self, msg):
        range_size = len(msg.ranges)
        possible_distance = reduce(lambda x,y: x+y, list(
            filter(lambda x: x >= 0, msg.ranges[range_size/2:range_size*3/4])))
        # 정면부터 우측까지의 거리를 합산하여 장애물 극복이 가능한 거리인지 탐지
        if 130 < possible_distance < 204:
            self.detect_obstacle_pub.publish(True)
        else:
            self.detect_obstacle_pub.publish(False)


if __name__ == "__main__":
    rospy.init_node('obstacle_test')
    detect_obstacle = DetectObstacle()
    while not rospy.is_shutdown():
        detect_obstacle.rate.sleep()


