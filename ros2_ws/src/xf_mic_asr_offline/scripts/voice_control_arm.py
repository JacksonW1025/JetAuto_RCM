#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/18
# @author:aiden
# 语音控制机械臂
import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from xf_mic_asr_offline import voice_play
from servo_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import BuzzerState
from servo_controller.action_group_controller import ActionGroupController

class VoiceControlColorDetectNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.language = os.environ['ASR_LANGUAGE']
        self.create_subscription(String, '/asr_node/voice_words', self.words_callback, 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        self.controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/share/arm_pc/ActionGroups')

        self.create_client(Trigger, '/asr_node/init_finish').wait_for_service()
        self.create_client(Trigger, '/ros_robot_controller/init_finish').wait_for_service()

        self.controller.run_action('init')
        self.play('running')

        self.get_logger().info('唤醒口令: 小幻小幻(Wake up word: hello hiwonder)')
        self.get_logger().info('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        self.get_logger().info('控制指令: 拔个萝卜 拿给我(Voice command: pick a carrot/pass me please)')
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
    
    def get_node_state(self, request, response):
        response.success = True
        return response

    def play(self, name):
        voice_play.play(name, language=self.language)

    def words_callback(self, msg):
        words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            words = words.replace(' ', '')
        self.get_logger().info('words: %s'%words)
        if words is not None and words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)', '失败10次(Fail-10-times']:
            if words == '拔个萝卜' or words == 'pick a carrot':
                self.play('ok')
                self.controller.run_action('voice_pick')
            elif words == '拿给我' or words == 'pass me please':
                self.play('come')
                self.controller.run_action('voice_give')
        elif words == '唤醒成功(wake-up-success)':
            self.play('awake')
        elif words == '休眠(Sleep)':
            msg = BuzzerState()
            msg.freq = 1900
            msg.on_time = 0.05
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)

def main():
    node = VoiceControlColorDetectNode('VoiceControMovelNode')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
