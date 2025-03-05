#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/11/18
# @author:aiden
# 语音控制移动
import os
import json
import math
import time
import rclpy
import threading
import numpy as np
import sdk.pid as pid
import sdk.common as common
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32
from xf_mic_asr_offline import voice_play
from ros_robot_controller_msgs.msg import BuzzerState

MAX_SCAN_ANGLE = 240  # 激光的扫描角度,去掉总是被遮挡的部分degree
CAR_WIDTH = 0.4  # meter

class VoiceControMovelNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.angle = None
        self.words = None
        self.running = True
        self.haved_stop = False
        self.lidar_follow = False
        self.start_follow = False
        self.last_status = Twist()
        self.threshold = 3
        self.speed = 0.3
        self.stop_dist = 0.4
        self.count = 0
        self.scan_angle = math.radians(45)

        self.pid_yaw = pid.PID(1.6, 0, 0.16)
        self.pid_dist = pid.PID(1.7, 0, 0.16)

        self.language = os.environ['ASR_LANGUAGE']
        self.lidar_type = os.environ.get('LIDAR_TYPE')
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.create_subscription(String, '/asr_node/voice_words', self.words_callback, 1)
        self.create_subscription(Int32, '/awake_node/angle', self.angle_callback, 1)

        self.client = self.create_client(Trigger, '/asr_node/init_finish')
        self.client.wait_for_service()  # 阻塞等待

        self.mecanum_pub.publish(Twist())
        self.play('running')

        self.get_logger().info('唤醒口令: 小幻小幻(Wake up word: hello hiwonder)')
        self.get_logger().info('唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        self.get_logger().info('控制指令: 左转 右转 前进 后退 过来(Voice command: turn left/turn right/go forward/go backward/come here)')

        self.time_stamp = time.time()
        self.current_time_stamp = time.time()

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def play(self, name):
        voice_play.play(name, language=self.language)

    def words_callback(self, msg):
        self.words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            self.words = self.words.replace(' ', '')
        self.get_logger().info('words:%s' % self.words)
        if self.words is not None and self.words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)',
                                                         '失败10次(Fail-10-times']:
            pass
        elif self.words == '唤醒成功(wake-up-success)':
            self.play('awake')
        elif self.words == '休眠(Sleep)':
            msg = BuzzerState()
            msg.freq = 1000
            msg.on_time = 0.1

            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)

    def angle_callback(self, msg):
        self.angle = msg.data
        self.get_logger().info('angle:%s' % self.angle)
        self.start_follow = False
        self.mecanum_pub.publish(Twist())

    def lidar_callback(self, lidar_data):
        twist = Twist()
        # 数据大小 = 扫描角度/每扫描一次增加的角度(data size= scanning angle/ the increased angle per scan)
        if self.lidar_type != 'G4':
            min_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
            max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[:max_index]  # 左半边数据 (the left data)
            right_ranges = lidar_data.ranges[::-1][:max_index]  # 右半边数据 (the right data)
        elif self.lidar_type == 'G4':
            min_index = int(math.radians((360 - MAX_SCAN_ANGLE) / 2.0) / lidar_data.angle_increment)
            max_index = int(math.radians(180) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[::-1][min_index:max_index][::-1]  # 左半边数据 (the left data)
            right_ranges = lidar_data.ranges[min_index:max_index][::-1]  # 右半边数据 (the right data)
        if self.start_follow:
            # 根据设定取数据 (get the data according to the settings)
            angle = self.scan_angle / 2
            angle_index = int(angle / lidar_data.angle_increment + 0.50)
            left_range, right_range = np.array(left_ranges[:angle_index]), np.array(right_ranges[:angle_index])

            # 拼合距离数据, 从右半侧逆时针到左半侧(the merged distance data from right half counterclockwise to the left half)
            ranges = np.append(right_range[::-1], left_range)
            nonzero = ranges.nonzero()
            nonan = ~np.isnan(ranges[nonzero])
            dist_ = ranges[nonzero][nonan]
            if len(dist_) > 1:
                dist = dist_.min()
                min_index = list(ranges).index(dist)
                angle = -angle + lidar_data.angle_increment * min_index  # 计算最小值对应的角度(calculate the angle corresponding to the minimum value)
                if dist < self.threshold and abs(math.degrees(angle)) > 5:  # 控制左右(control the left and right)
                    if self.lidar_type != 'G4':
                        self.pid_yaw.update(-angle)
                        twist.angular.z = common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0)
                    elif self.lidar_type == 'G4':
                        self.pid_yaw.update(angle)
                        twist.angular.z = -common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0)
                else:
                    self.pid_yaw.clear()

                if dist < self.threshold and abs(self.stop_dist - dist) > 0.02:  # 控制前后(control the front and back)
                    self.pid_dist.update(self.stop_dist - dist)
                    twist.linear.x = common.set_range(self.pid_dist.output, -self.speed, self.speed)
                else:
                    self.pid_dist.clear()

                if abs(twist.angular.z) < 0.008:
                    twist.angular.z = 0.0
                if abs(twist.linear.x) < 0.05:
                    twist.linear.x = 0.0
                if twist.linear.x == 0 and twist.angular.z == 0:
                    self.count += 1
                if self.count >= 10:
                    self.count = 0
                    self.start_follow = False
                self.mecanum_pub.publish(twist)

    def main(self):
        while True:
            if self.words is not None:
                twist = Twist()
                if self.words == '前进' or self.words == 'go forward':
                    self.play('go')
                    self.time_stamp = time.time() + 2
                    twist.linear.x = 0.2
                elif self.words == '后退' or self.words == 'go backward':
                    self.play('back')
                    self.time_stamp = time.time() + 2
                    twist.linear.x = -0.2
                elif self.words == '左转' or self.words == 'turn left':
                    self.play('turn_left')
                    self.time_stamp = time.time() + 2
                    twist.angular.z = 0.8
                elif self.words == '右转' or self.words == 'turn right':
                    self.play('turn_right')
                    self.time_stamp = time.time() + 2
                    twist.angular.z = -0.8
                elif self.words == '过来' or self.words == 'come here':
                    self.play('come')
                    if 270 > self.angle > 90:
                        twist.angular.z = -1.0
                        self.time_stamp = time.time() + abs(math.radians(self.angle - 90) / twist.angular.z)
                    else:
                        twist.angular.z = 1.0
                        if self.angle <= 90:
                            self.angle = 90 - self.angle
                        else:
                            self.angle = 450 - self.angle
                        self.time_stamp = time.time() + abs(math.radians(self.angle) / twist.angular.z)
                    # self.get_logger().info(self.angle)
                    self.lidar_follow = True
                elif self.words == '休眠(Sleep)':
                    time.sleep(0.01)
                self.words = None
                self.haved_stop = False
                self.mecanum_pub.publish(twist)
            else:
                time.sleep(0.01)
            self.current_time_stamp = time.time()
            if self.time_stamp < self.current_time_stamp and not self.haved_stop:
                self.mecanum_pub.publish(Twist())
                self.haved_stop = True
                if self.lidar_follow:
                    self.lidar_follow = False
                    self.start_follow = True

def main():
    node = VoiceControMovelNode('voice_control_move')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
