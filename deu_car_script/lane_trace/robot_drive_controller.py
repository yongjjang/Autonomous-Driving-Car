#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


class RobotDriveController:

    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.rate = rospy.Rate(10)
        self.velocity = 0
        self.angular = 0

    def set_velocity(self, velocity):
        self.velocity = velocity

    def set_angular(self, angular):
        self.angular = angular

    def drive_forward(self, velocity):
        self.twist.linear.x = velocity
        self.cmd_vel_pub.publish(self.twist)

    def turn_angular(self, degree):
        self.twist.angular.z = degree
        self.cmd_vel_pub.publish(self.twist)


if __name__ == "__main__":
    robotDrivenController = RobotDriveController()
    robotDrivenController.drive_forward(1)

    while not rospy.is_shutdown():
        robotDrivenController.cmd_vel_pub.publish(robotDrivenController.twist)
        robotDrivenController.rate.sleep()

