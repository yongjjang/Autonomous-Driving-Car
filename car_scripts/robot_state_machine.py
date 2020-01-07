#!/usr/bin/env python


import rospy
import robot_state
from smach import StateMachine
import smach_ros


class RobotStateMachine(object):
    def __init__(self):
        self.autonomous_drive = StateMachine(outcomes=['success'])

    def drive_robot(self):
        with self.autonomous_drive:
            StateMachine.add('READY_TO_START', robot_state.ReadyToStart(), transitions={'success':'DETECT_BLOCKING_BAR'})
            StateMachine.add('DETECT_BLOCKING_BAR', robot_state.DetectedBlockingBar(),
                             transitions={'success': 'LANE_TRACE'})
            StateMachine.add('LANE_TRACE', robot_state.LaneTrace(),
                             transitions={'success': 'PROJECT_END', 'detected_stop_line': 'DETECT_STOP_LINE', 'detected_stop_sign': 'DETECT_STOP_SIGN', 'detected_crossroad': 'DETECT_CROSSROAD','detected_parking_zone': 'DETECT_PARKING_ZONE', 'detected_obstacle' : 'DETECT_OBSTACLE'})
            StateMachine.add('DETECT_STOP_LINE', robot_state.DetectedStopLine(), transitions={'success': 'LANE_TRACE'})
            StateMachine.add('DETECT_STOP_SIGN', robot_state.DetectedStopSign(), transitions={'success': 'LANE_TRACE'})
            StateMachine.add('DETECT_PARKING_ZONE', robot_state.DetectedParkingZone(), transitions={'success': 'LANE_TRACE'})
            StateMachine.add('DETECT_CROSSROAD', robot_state.DetectedCrossroad(), transitions={'success': 'LANE_TRACE'})
            StateMachine.add('DETECT_OBSTACLE', robot_state.DetectedObstacle(), transitions={'success': 'LANE_TRACE'})
            StateMachine.add('PROJECT_END', robot_state.ProjectEnd(), transitions={'success': 'success'})
            # StateMachine.add('') ....

        sis = smach_ros.IntrospectionServer('test', self.autonomous_drive, '/SM_ROOT')
        sis.start()
        self.autonomous_drive.execute()
        sis.stop()


if __name__ == "__main__":
    rospy.init_node('autonomous_car_test_map_drive')
    robot_state_machine = RobotStateMachine()
    robot_state_machine.drive_robot()
    while not rospy.is_shutdown():
        rospy.spin()
