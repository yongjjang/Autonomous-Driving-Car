#! /usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from base_detector import BaseDetector


class DetectParkingZone(BaseDetector):
    """
    주차장을 인식하고 해당 토픽을 발행하는 클래스

    """
    def __init__(self):
        super(DetectStopSign, self).__init__()
        pass

    def image_callback(self, msg):
        pass


if __name__ == "__main__":
    pass