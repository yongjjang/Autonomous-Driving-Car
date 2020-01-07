#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist


class RobotDriveController:
    """
    로봇 주행을 위한 선속도와 회전속도를 제어하는 클래스
    """
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self._velocity = 0
        self._angular = 0

    def set_velocity(self, velocity):
        """
        :param velocity : std_msgs.msg Twist.linear.x
        터틀봇의 주행속도를 설정, velocity : 1일 경우, 1m/sec단위로 동작함.
        :return:
        """
        self._velocity = velocity

    def set_angular(self, angular):
        """
        :param angular: std_msgs.msg Twist.angular.z
        터틀봇의 각속도를 설정
        :return:
        """
        self._angular = angular

    def drive(self):
        self.twist.linear.x = self._velocity
        self.twist.angular.z = self._angular
        self.cmd_vel_pub.publish(self.twist)


if __name__ == "__main__":
    rospy.init_node('robot_drive_controller')
    robotDriveController = RobotDriveController()
    robotDriveController.set_velocity(1)
    robotDriveController.drive()
    rospy.spin()
