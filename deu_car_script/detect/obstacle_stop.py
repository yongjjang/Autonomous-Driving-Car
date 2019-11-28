#!/usr/bin/env python
# coding=utf-8

import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from robot_drive_controller import RobotDriveController


# scan[len(scan/2)] = ㅠㅠ
class DetectObstacle:
    def __init__(self):
        self.range_ahead = 0
        self.range_right = 0
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.stop_pub = rospy.Publisher('stop_sign', Bool, queue_size=1)
        self.drive_controller = RobotDriveController()

    def scan_callback(self, msg):
        angle_30 = len(msg.ranges) / 12
        angle_45 = len(msg.ranges) / 8

        # msg.ranges / 2 = range_ahead
        self.range_ahead = msg.ranges[angle_45 * 4]
        # right = 210 ~ 255의 최댓값
        self.range_right = max(msg.ranges[(angle_45 * 4) + angle_30: (angle_45 * 5) + angle_30])

        # 정면 물체, 측면 물체까지의 거리 출력
        print "range ahead : %0.2f" % self.range_ahead
        print "range right : %0.2f" % self.range_right

        #  something_else = True if (math.isnan(self.range_ahead) and math.isnan(self.range_right)) or self.range_right > 1.0 else False

        if (math.isnan(self.range_ahead) and math.isnan(self.range_right)) or (self.range_right > 2.5 and self.range_ahead > 2.4):
            value = False
            self.stop_pub.publish(value)
            self.drive_controller.drive_forward(1)
            print('go')
        else:
            value = True
            self.stop_pub.publish(value)
            self.drive_controller.set_velocity(0)
            print('stop')


if __name__ == "__main__":
    rospy.init_node('obstacle_test')
    detect_obstacle = DetectObstacle()
    rospy.spin()
