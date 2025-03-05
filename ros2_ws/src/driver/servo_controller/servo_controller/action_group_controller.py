#!/usr/bin/env python3
# encoding: utf-8
import os
import time
import sqlite3 as sql
from servo_controller_msgs.msg import ServosPosition, ServoPosition

class ActionGroupController:
    running_action = False
    stop_running = False
    def __init__(self, pub, action_path):
        self.servo_controller_pub = pub
        self.action_path = action_path

    def stop_action_group(self):
        self.stop_running = True

    def run_action(self, action_name):
        '''
        运行动作组，无法发送stop停止信号
        :param action_name: 动作组名字 ， 字符串类型
        :param times:  运行次数
        :return:
        '''
        if action_name is None:
            return
        action_name = os.path.join(self.action_path, action_name + ".d6a")
        self.stop_running = False
        if os.path.exists(action_name) is True:
            if self.running_action is False:
                self.running_action = True
                ag = sql.connect(action_name)
                cu = ag.cursor()
                cu.execute("select * from ActionGroup")
                while True:
                    act = cu.fetchone()
                    if self.stop_running is True:
                        self.stop_running = False                   
                        break
                    if act is not None:
                        data = []
                        msg = ServosPosition()
                        msg.duration = float(act[1])/1000.0
                        for i in range(0, len(act)-2, 1):
                            servo = ServoPosition()
                            if i + 1 == 6:
                                servo.id = 10
                            else:
                                servo.id = i + 1
                            servo.position = int(act[2 + i])
                            data.append(servo)
                        msg.position = data
                        self.servo_controller_pub.publish(msg)
                        time.sleep(float(act[1])/1000.0)
                    else:   # 运行完才退出
                        break
                self.running_action = False
                
                cu.close()
                ag.close()
        else:
            self.running_action = False
            print("未能找到动作组文件")
