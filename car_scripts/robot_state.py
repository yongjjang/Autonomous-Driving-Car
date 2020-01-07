#!/usr/bin/env python
# coding=utf-8

import rospy
from smach import State
from lane_tracer import LaneTracer
from robot_drive_controller import RobotDriveController
from detect_stop_sign import DetectStopSign
from detect_stop_line import DetectStopLine
from detect_blocking_bar import DetectBlockingBar
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import LaserScan


class ReadyToStart(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.drive_controller = RobotDriveController()

    def execute(self, ud):
        rospy.loginfo("시작 전 대기 중.")
        rospy.sleep(rospy.Duration(3))
        return 'success'


class DetectedBlockingBar(State):
    """
    차단바가 인식되었을 경우, 이 상태로 천이됨.
    차단바를 지나치고 난 후, 주행(LaneTrace)상태로 복귀

    """
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.detect_blocking_bar = DetectBlockingBar()
        self.drive_controller = RobotDriveController()
        self.rate = rospy.Rate(20)

    def execute(self, ud):
        current_time = int(rospy.Time.now().to_sec())
        target_time = current_time + 3

        rospy.loginfo("차단바 열림이 인식되었음.")
        while target_time > int(rospy.Time.now().to_sec()):
            self.drive_controller.set_velocity(1)
            self.drive_controller.set_angular(0)
            self.drive_controller.drive()
            return 'success'


class LaneTrace(State):
    """
    해당 상태는 기본 주행상태임.
    주행 중 탐지되는 이벤트가 발생할 경우, 이벤트에 대응하는 행동 상태로 천이함.
    """
    def __init__(self):
        State.__init__(self, outcomes=['success', 'detected_stop_line', 'detected_obstacle',
                                       'detected_stop_sign', 'detected_crossroad', 'detected_parking_zone'])
        self.left_line = LaneTracer('my_left_camera/rgb/image_raw')
        self.right_line = LaneTracer('my_right_camera/rgb/image_raw')
        self.drive_controller = RobotDriveController()

        self.detect_stop_line = DetectStopLine()
        self.detect_stop_sign = DetectStopSign()

        self.is_stop_line = rospy.Subscriber('detect/is_stop_line', Bool, self.stop_line_callback)
        self.is_stop_sign = rospy.Subscriber('detect/is_stop_sign', Bool, self.stop_sign_callback)

        self.is_stop_line = False
        self.is_stop_sign = False
        self.is_parking_zone = False
        self.is_crossroad = False
        self.is_success = False
        self.is_obstacle = False

    def stop_line_callback(self, msg):
        if msg.data:
            self.is_stop_line = True
        else:
            self.is_stop_line = False

    def stop_sign_callback(self, msg):
        if msg.data:
            self.is_stop_sign = True
        else:
            self.is_stop_sign = False

    def execute(self, ud):
        rospy.loginfo("차선 주행 모드로 전환")
        while True:
            """
            주행 중 이벤트 발생 시, 이벤트에 대응하는 상태로 천이를 위한 반복문 
            """
            # Transition Start
            if self.is_stop_line:
                self.is_stop_line = not self.is_stop_line
                return 'detected_stop_line'
            if self.is_stop_sign:
                self.is_stop_sign = not self.is_stop_sign
                return 'detected_stop_sign'
            if self.is_parking_zone:
                self.is_parking_zone = not self.is_parking_zone
                return 'detected_parking_zone'
            if self.is_crossroad:
                self.is_crossroad = not self.is_crossroad
                return 'detected_crossroad'
            if self.is_obstacle:
                self.is_obstacle = not self.is_obstacle
                return 'detected_obstacle'
            if self.is_success:
                return 'success'
            # Transition End

            cx = (self.left_line.cx + self.right_line.cx) / 2
            err = -float(cx) / 80
            if abs(err) > 0.14:
                self.drive_controller.set_velocity(0.4)
                self.drive_controller.set_angular(err)
            elif abs(err) < 0.14:
                self.drive_controller.set_velocity(1)
                self.drive_controller.set_angular(err)
            self.drive_controller.drive()


class DetectedStopLine(State):
    def __init__(self):
        """
        카메라에 정지선이 탐지되었을 경우의 상태를 의미함.
        정지선 앞에서 3초간 정지 후, 다시 기본주행 상태로 복귀함.
        """
        State.__init__(self, outcomes=['success'])
        self.drive_controller = RobotDriveController()
        self.stop_line_sub = rospy.Subscriber('detect/stop_line_cx', Float32, self.stop_line_callback)
        self.rate = rospy.Rate(20)
        self.count = 0

    def stop_line_callback(self, msg):
        self.cx = msg.data

    def execute(self, ud):
        rospy.loginfo("정지선이 인식되었음.")

        time_now = int(rospy.Time.now().to_sec())
        target_time = time_now + 3

        while target_time > int(rospy.Time.now().to_sec()):
            self.drive_controller.set_velocity(0)
            self.rate.sleep()
        return 'success'


class DetectedStopSign(State):
    """
    정지 표지판이 인식되었을 경우,  이 상태로 천이됨.
    정지 표지판 앞에서 3초 간 정지 후, 기본 주행상태로 복귀함.
    """
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.drive_controller = RobotDriveController()
        self.count = 0
        self.rate = rospy.Rate(20)

    def execute(self, ud):
        rospy.loginfo("정지 표지판이 인식되었음.")

        time_now = int(rospy.Time.now().to_sec())
        target_time = time_now + 3

        while target_time > int(rospy.Time.now().to_sec()):
            self.drive_controller.set_velocity(0)
            self.rate.sleep()

        if self.count is 2:
            return 'thank_for_you_turtle'
        else:
            return 'success'


class DetectedCrossroad(State):
    """
    교차로에 진입하였을 경우 해당 상태로 천이됨.
    상황에 따라 직진 혹은 우회전 후, 기본 주행상태로 복귀함.
    """
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.drive_controller = RobotDriveController()

    def execute(self, ud):
        rospy.loginfo("교차로 상태 진입")
        current_time = int(rospy.Time.now().to_sec())
        target_time = current_time + 3

        while target_time > int(rospy.Time.now().to_sec()):
            self.drive_controller.set_velocity(1)
            self.drive_controller.drive()

        return 'success'


class DetectedParkingZone(State):
    """
    주차구간을 인식하였을 경우 해당 상태로 천이됨.
    T자 코스 또는 평행 주차 코스에 대응하는 행동 후, 기본 주행상태로 복귀함.
    """
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.drive_controller = RobotDriveController()

    def execute(self, ud):
        rospy.loginfo("주차 상태 진입")
        return 'success'


class DetectedObstacle(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.drive_controller = RobotDriveController()
        self.detect_obstacle_sub = rospy.Subscriber('detect/is_obstacle', LaserScan, callback=self.scan_callback)

    def scan_callback(self, msg):
        pass

    def execute(self, ud):
        rospy.loginfo("장애물이 인식되었음")
        self.drive_controller.set_velocity(0)
        return 'success'


class ProjectEnd(State):
    """
    마지막 코스에 도달하였을 경우 해당 상태로 천이됨.
    해당 상태가 종료되면, 상태 기계가 종료됨.
    """

    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.drive_controller = RobotDriveController()
        self.rate = rospy.Rate(20)

    def execute(self, ud):
        rospy.loginfo("프로젝트 종료")
        while rospy.is_shutdown():
            self.drive_controller.set_velocity(7)
            self.drive_controller.set_angular(1)
            self.drive_controller.drive()
            print "Yeah!!!!!!"
        return 'success'

