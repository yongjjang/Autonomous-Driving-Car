#!/usr/bin/env python
# -*- coding: utf-8 -*-


class DetectBehavior:
    """
    카메라가 특수한 행동을 감지했을 경우, 감지된 대상에 따라 작동을 변경하는 클래스
    """
    def __init__(self, detect_method):
        self.detect_method = detect_method

        """
        :param detect_method: DetectMethod
        detect_method.py 파이썬스크립트의 DetectMethod 클래스를 호출함. 
        """

    def set_detect_behavior(self, detect_method):
        self.detect_method = detect_method

    def do_behavior(self):
        self.detect_method.do_behavior()
