#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
#Title: 2019 RoboCupJapanOpen GoGetItのマスター用ROSノード
#Author: Issei Iida
#Date: 2019/10/13
#Memo: 
#---------------------------------------------------------------------

#Python関係
import sys
#ROS関係
import rospy
import smach
import smach_ros
from std_msgs.msg import String

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_action_client import enterTheRoomAC
from common_function import *


######################################################################
# Door Open Start
######################################################################


class Admission(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['entered_room'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: ADMISSION')
        result = enterTheRoomAC()
        return 'entered_room'


######################################################################
# Training Phase
######################################################################


class listenCommand(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['follow',
                            'move',
                            'learn',
                            'finish'],
                input_keys=['learn_data_input'],
                output_keys=['li_command_output'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN_COMMAND')
        #コマンドを認識する処理
        rospy.loginfo('Command is <' + str(command) + '>')
        userdata.li_command_output = command
        if command is 'start_follow' or 'stop_follow':
            return 'follow'
        elif command is 'turn_right' or 'turn_left' or 'go_back':
            return 'move'
        elif command is 'learn_start':
            return 'learn'
        elif command is 'finish training':
            rospy.loginfo('Create LearnData Param')
            rospy.set_param('/ggi/lean_data', userdata.learn_data_input)
            rosparam.dump_prarams('/hone/issei/catkin_ws/src/ja_2019_ggi/config/learn_data.yaml', '/ggi/learn_data')
            rospy.loginfo('Created LearnData!')
            return 'finish'


class follow(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_follow'],
                input_keys = ['f_command_input'])
        #Publisher
        self.pub_follow_req = rospy.Publisher('/chase/request', String, queue_size = 1)

    def execute(self, userdata):
        rospy.loginfo('Executing state: FOLLOW')
        if userdata.f_command_input == 'follow':
            rospy.loginfo('Start follow')
            speak('I will follow you')
            #self.pub_follow_req.publish('start')
        elif userdata.f_command_input == 'stop_follow':
            rospy.loginfo('Stop follow')
            speak('Stop following')
            #self.pub_follow_req.publish('stop')
        return 'finish_follow'


class move(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_move'],
                input_keys = ['m_command_input'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVE')
        if userdata.m_command_input == 'turn_right':
            rospy.loginfo('Turn right')
            speak('Rotate right')
        elif userdata.m_command_input == 'turn_left':
            rospy.loginfo('Turn left')
            speak('Rotate left')
        return 'finish_move'


class learn(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_learn'],
                input_keys = ['le_command_input'],
                output_keys = ['learn_data_output'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: LEARN')
        rospy.loginfo('Start learning')
        #データ構造と追加方法は未定
        #learn_data.append()
        #userdata.learn_data_output = learn_data


######################################################################
# Test Phase
######################################################################


class listenOrder(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['order_req_success',
                            'order_req_failed',
                            'finish_all_order'],
                output_keys = ['l_order_output'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN_ORDER')
        #Order聞く処理を追加
        order_data = []
        order_data = ['shelf',
                      'cupnoodle']
        userdata.l_order_output = order_data


class executeOrder(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_order'],
                input_keys = ['e_order_input'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: EXECUTE_ORDER')
        #ナビゲーションから物体把持でナビゲーションそして渡す
        location = userdata.e_order_input[0]
        target_object = userdata.e_order_input[1]
        rospy.loginfo('Start navigation')
        searchLocationName(location)
        startNavigation()
        rospy.loginfo('Start Grasp')


######################################################################
# StateMachine
######################################################################


def main():
    sm_top = smach.StateMachine(
            outcomes = ['finish_ggi'])
    with sm_top:
        smach.StateMachine.add(
                'ADMISSION',
                Admission(),
                transitions = {'entered_room':'TRAINING_PHASE'})

        sm_training = smach.StateMachine(
                outcomes = ['finish_training'])
        with sm_training:
            smach.StateMachine.add(
                    'LISTEN_COMMAND',
                    ListenCommand(),
                    transitions = {'follow':'FOLLOW',
                                   'move':'MOVE',
                                   'learn':'LEARN',
                                   'finish':'finish_training'},
                    remapping = {'li_command_output':'command_name',
                                 'learn_data_input':'learn_data'})

            smach.StateMachine.add(
                    'FOLLOW',
                    Follow(),
                    transitions = {'finish_follow':'LISTEN_COMMAND'},
                    remapping = {'f_command_input':'command_name'})

            smach.StateMachine.add(
                    'MOVE',
                    Move(),
                    transitions = {'finish_move':'LISTEN_COMMAND'},
                    remapping = {'m_command_input':'command_name'})

            smach.StateMachine.add(
                    'LEARN',
                    Learn(),
                    transitions = {'finish_learn':'LISTEN_COMMAND'},
                    remapping = {'le_command_input':'command_name',
                                  'learn_data_output':'learn_data'})

        smach.StateMachine.add(
                'TRAINING_PHASE',
                sm_training,
                transitions = {'finish_training':'TEST_PHASE'},
                remapping = {'t_command_input':'command_name',
                             't_command_output':'command_name'})

        sm_test = sm.StateMachine(
                outcomes = ['finish_test'])
        with sm_test:
            smach.StateMachine.add(
                    'LISTEN_ORDER',
                    ListenOrder(),
                    transitions = {'order_req_success':'EXECUTE_ORDER',
                                   'order_req_failed':'LISTEN_ORDER',
                                   'finish_all_order':'finish_test'},
                    remapping = {'l_order_output':'order_data'})

            smach.StateMachine.add(
                    'EXECUTE_ORDER',
                    ExecuteOrder(),
                    transitions = {'finish_order':'LISTEN_ORDER'},
                    remapping = {'e_order_input':'order_data'})

        smach.StateMachine.add(
                'TEST_PHASE',
                sm_test,
                transitions = {'finish_test':'finish_ggi'})

    outcome = sm_top.execute()

if __name__ == '__main__':
    try:
        rospy.init_node('sm_ggi', anonymous = True)
        main()
    except rospy.ROSInterrutException:
        pass
