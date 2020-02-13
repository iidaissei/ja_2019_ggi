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


class ListenOrder(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['listen_success',
                                         'listen_failed',
                                         'all_finish'],
                             output_keys = ['action_out',
                                            'data_out'])
        # Service
        # self.order_req_srv = ''
        # Value
        self.listen_count = 1

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN_ORDER')
        if self.listen_count <= 3:
            # print self.listen_count
            result = False
            # result = self.order_req_srv()
            # if result.result:
            if result:
                # userdata.action_out = result.action
                # userdata.data_out = result.data
                userdata.action_out = ['go', 'grasp']
                userdata.data_out = ['shelf', 'cup']
                self.listen_count = 1
                return 'listen_success'
            else:
                # speak('Listening failed')
                self.listen_count += 1
                print self.listen_count
                return 'listen_failed'
        else:
            self.listen_count = 1
            return 'all_finish'


class OrderCount(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['not_complete',
                                         'all_complete'])
        self.order_count = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state: ORDER_COUNT')
        if self.order_count < 3:
            self.order_count += 1
            rospy.loginfo('Order num: ' + str(self.order_count))
            return 'not_complete'
        else:
            rospy.loginfo('All order completed!')
            return 'all_complete'


class ExeAction(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['action_success',
                                         'action_failed'],
                             input_keys = ['action_in',
                                           'data_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: EXE_ACTION')
        a_list = userdata.action_in
        d_list = userdata.data_in
        # result = exeActionPlanAC(a_list, d_list)
        result = True
        # if result.result:
        if result:
            return 'action_success'
        else:
            return 'action_failed'


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
                'ORDER_COUNT',
                OrderCount(),
                transitions = {'not_complete':'LISTEN_ORDER',
                               'all_complete':'EXIT'})

        smach.StateMachine.add(
                'LISTEN_ORDER',
                ListenOrder(),
                transitions = {'listen_success':'EXE_ACTION',
                               'listen_failed':'LISTEN_ORDER',
                               'all_finish':'ORDER_COUNT'},
                remapping = {'action_out':'action_list',
                             'data_out':'data_list'})

        # smach.StateMachine.add(
        #         'ORDER_COUNT',
        #         OrderCount(),
        #         transitions = {'not_complete':'LISTEN_ORDER',
        #                        'all_complete':'EXIT'})

        smach.StateMachine.add(
                'EXE_ACTION',
                ExeAction(),
                transitions = {'action_success':'ORDER_COUNT',
                               'action_failed':'ORDER_COUNT'},
                remapping = {'action_in':'action_list',
                             'data_in':'data_list'})

        smach.StateMachine.add(
                'EXIT',
                Exit(),
                transitions = {'finish_exit':'finish_sm_top'})

    outcomes = sm_top.execute()


if __name__ == '__main__':
    rospy.init_node('sm_test_phase', anonymous = True)
    main()
