#!/usr/bin/env python
# -*- coding:utf-8 -*-
'''
작성자 : 신휘정
작성일자 : 2019.11.19
용도 : Left, Right 차선 오차값 subscribe하고 이동 값 publish
TODO
'''
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from Ros01team02.msg import DriveLineInfo

'''
subscribe : Left, right moment error value
publish : movement value
'''
class LineSubscriber:
    def __init__(self):

        self.leftLine_sub = rospy.Subscriber('DriveLineInfo',
                                          DriveLineInfo, self.set_movement)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                           Twist, queue_size=1)
        self.twist = Twist()

    def set_movement(self,msg):
        err = msg.leftLine + msg.rightLine

        rospy.loginfo(err)

        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 2000
        self.cmd_vel_pub.publish(self.twist)

rospy.init_node('LineSubscriber')
lineSubscriber = LineSubscriber()
rospy.spin()
