#!/usr/bin/env python
# coding=utf-8

import abc
import cv_bridge
from sensor_msgs.msg import Image
import rospy


class BaseDetector:
    """
    디텍터 함수의 구조를 추상 메소드를 통해 정의하고,
    반복해서 선언해야 하는 속성들의 중복을 최소화 하기 위한 부모 클래스
    """
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()  # OpenCV2 와 ROS의 상호변환을 돕는 패키지
        self.current_focus_pub = rospy.Publisher('current_focus_image', Image, queue_size=1)
        # 현재 발생하는 이벤트에 대한 이미지 발행
        self.rate = rospy.Rate(20)

    @abc.abstractmethod
    def image_callback(self, msg):  # 이미지 토픽을 사용한다고 알림
        """
        :type msg: Image, scan
        """
        raise NotImplementedError()
