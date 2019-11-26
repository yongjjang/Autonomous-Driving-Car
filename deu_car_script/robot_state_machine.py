#!/usr/bin/env python


import rospy
from smach import State, StateMachine


if __name__ == "__main__":
    sm = StateMachine(outcomes=['success']);
    with sm:
        StateMachine.add('DETECT_BLOCKING_BAR', DetectBlockingBar(), transitions=['success':"LANE_TRACE"])
        StateMachine.add('LANE_TRACE', LaneTrace(), transitions=['success':'CROSSROADS'])
        StateMachine.add('CROSSROADS_DRIVE', CrossroadsDrive(), transitions=['success':''])
        # StateMachine.add('') ....





    sm.execute()
