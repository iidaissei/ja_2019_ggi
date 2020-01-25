#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------------------------------
# Title: 2019 RoboCupJapanOpen GoGetItのマスター用ROSノード
# Author: Issei Iida
# Date: 2019/10/13
# Memo: 
#---------------------------------------------------------------------

# Python
import sys
# ROS
import rospy
import smach
import smach_ros
from std_msgs.msg import String
from ggi.srv import ListenCommand, GgiLearning

sys.path.insert(0, '/home/athome/catkin_ws/src/mimi_common_pkg/scripts')
from common_action_client import enterTheRoomAC
from common_function import *

class Admission(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['entered_room'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: ADMISSION')
        #result = enterTheRoomAC()
        return 'entered_room'

#-----------------------------------------------
# TrainingPhase
#-----------------------------------------------

class ListenCmd(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['follow',
                            'move',
                            'learn',
                            'finish',
                            'listen_cmd_failed'],
                input_keys=['learn_data_input'],
                output_keys=['li_command_output'])
        # Service
        self.listen_cmd_srv = rospy.ServiceProxy('/listen_command', ListenCommand)

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN_CMD')
        result = self.listen_cmd_srv()
        print result
        if result.result == True:
            command = result.cmd
            userdata.cmd_output = command
            rospy.loginfo('Command is *' + str(command) + '*')
            if command in self.cmd_dict:
                key = self.cmd_dict[command]
                return key
            elif command == 'finish_training':
                return 'finish_training'
            else:
                rospy.loginfo(str(command) + ' is not found')
                return 'listen_cmd_failed'
        else:
            rospy.loginfo('Listening failed')
            return 'listen_cmd_failed'

class Follow(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_follow'],
                input_keys = ['f_command_input'])
        # Publisher
        self.pub_follow_req = rospy.Publisher('/chase/request', String, queue_size = 1)

    def execute(self, userdata):
        rospy.loginfo('Executing state: FOLLOW')
        if userdata.f_command_input == 'start_follow':
            rospy.loginfo('Start follow')
            speak('I will follow you')
            self.pub_follow_req.publish('start')
        elif userdata.f_command_input == 'stop_follow':
            rospy.loginfo('Stop follow')
            speak('Stop following')
            self.pub_follow_req.publish('stop')
        return 'finish_follow'


class Move(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_move'],
                input_keys = ['m_command_input'])
        self.bc = BaseCarrier()

    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVE')
        if userdata.m_command_input == 'turn_right':
            rospy.loginfo('Turn right')
            speak('Rotate right')
            self.bc.angleRotation(-90)
        elif userdata.m_command_input == 'turn_left':
            rospy.loginfo('Turn left')
            speak('Rotate left')
            self.bc.angleRotation(90)
        elif userdata.m_command_input == 'go_straight':
            rospy.loginfo('Go straight')
            speak('Go forward')
            self.bc.translateDist(0.15)
        elif userdata.m_command_input == 'go_back':
            rospy.loginfo('Go back')
            speak('Go back')
            self.bc.translateDist(-0.15)
        else:
            pass
        return 'finish_move'


class Learn(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['finish_learn'],
                input_keys = ['le_command_input'],
                output_keys = ['learn_data_output'])
        # Survice
        self.ggi_learning_srv = rospy.ServiceProxy('/ggi_learning',
                                                   GgiLearning)

    def execute(self, userdata):
        rospy.loginfo('Executing state: LEARN')
        self.ggi_learning_srv()
        rospy.sleep(1.0)
        return 'finish_learn'

#-----------------------------------------------
# TestPhase
#-----------------------------------------------

class ListenOrder(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes = ['order_req_success',
                            'order_req_failed',
                            'finish_all_order'],
                output_keys = ['l_order_output'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: LISTEN_ORDER')
        # Order聞く処理を追加
        speak('Please give me a order')
        order_data = 'desk'
        userdata.l_order_output = order_data


class ExecuteOrder(smach.State):
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
        coord_list = searchLocationName('ggi_location', location)
        startNavigation(coord_list)
        rospy.loginfo('Start Grasp')

#-----------------------------------------------
# StateMachine
#-----------------------------------------------

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
                    ListenCmd(),
                    transitions = {'follow':'FOLLOW',
                                   'move':'MOVE',
                                   'learn':'LEARN',
                                   'finish':'finish_training',
                                   'listen_cmd_failed':'LISTEN_COMMAND'},
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

        sm_test = smach.StateMachine(
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
    rospy.init_node('sm_ggi', anonymous = True)
    main()
