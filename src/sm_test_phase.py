#!/usr/bin/env python
# -*- coding: utf-8 -*-
#----------------------------------------------------------
# Title: GoGetIt TestPhase用スクリプト
# Author: Issei Iida
# Date: 2020/02/12
# Memo: オーダーはBringmeなので、現在位置確認処理は省く
#----------------------------------------------------------

# Python
import sys
# ROS
import rospy
import smach
import smach_ros

sys.path.insert(0, '/home/issei/catkin_ws/src/mimi_common_pkg/scripts')
# from common_action_client import *
# from common_function import *


class ListenCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['listen_success',
                                         'listen_failure',
                                         'next_cmd',
                                         'all_cmd_finish'],
                             output_keys = ['cmd_out'])
        # ServiceProxy
        self.listen_srv = rospy.ServiceProxy('/gpsr/actionplan', ActionPlan)
        # Value
        self.listen_count = 1
        self.cmd_count = 1

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN_COMMAND')
        if self.cmd_count == 4:
            speak('Finish all command')
            return 'all_cmd_finish'
        elif self.listen_count <= 3:
            speak('CommandNumber is ' + str(self.cmd_count))
            speak('ListenCount is ' + str(self.listen_count))
            speak('Please instruct me')
            result = self.listen_srv()
            if result.result:
                self.listen_count = 1
                self.cmd_count += 1
                userdata.cmd_out = result
                return 'listen_success'
            else:
                self.listen_count += 1
                speak("Sorry, I could't listen")
                return 'listen_failure'
        else:
            speak("I couldn't understand the instruction")
            self.listen_count = 1
            self.cmd_count +=1
            return 'next_cmd'


class ExeAction(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['action_success',
                                         'action_failure'],
                             input_keys = ['cmd_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: EXE_ACTION')
        action = userdata.cmd_in.action
        data = userdata.cmd_in.data
        print data
        print action
        # result = exeActionPlan(action, data)
        result = True
        # if result.result:
        if result:
            speak('Action success')
            return 'action_success'
        else:
            speak('Action failed')
            return 'action_failure'


class Exit(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['finish_exit'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: EXIT')
        # coord_list = searchLocationName('', 'exit')
        # result = navigationAC(coord_list)
        result = True
        if result:
            return 'finish_exit'
        else:
            # speak('I can not mave exit')
            print 'exit'
            return 'finish_exit'


def main():
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])

    with sm_top:
        smach.StateMachine.add(
                'LISTEN_COMMAND',
                ListenCommand(),
                transitions = {'listen_success':'EXE_ACTION',
                               'listen_failure':'LISTEN_COMMAND',
                               'next_cmd':'DECIDE_MOVE',
                               'all_cmd_finish':'EXIT'},
                remapping = {'cmd_out':'cmd'})

        smach.StateMachine.add(
                'EXE_ACTION',
                ExeAction(),
                transitions = {'action_success':'DECIDE_MOVE',
                               'action_failure':'DECIDE_MOVE'},
                remapping = {'cmd_in':'cmd'})

        smach.StateMachine.add(
                'EXIT',
                Exit(),
                transitions = {'exit_finish':'finish_sm_top'})

    outcomes = sm_top.execute()


if __name__ == '__main__':
    rospy.init_node('sm_test_phase', anonymous = True)
    main()
