#!/usr/bin/env python3
# coding=utf-8
# @Author: Aiden
# @Date: 2023/11/16
# 环形麦克风阵列Python SDK
import re
import time
import json
import rclpy
import serial
import threading
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Int32, Bool
from xf_mic_asr_offline_msgs.srv import SetString

class CircleMic:
    def __init__(self, port, flag_pub, angle_pub):
        self.serialHandle = serial.Serial(None, 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout=0.02)
        self.serialHandle.rts = False
        self.serialHandle.dtr = False
        self.serialHandle.setPort(port)
        self.serialHandle.open()

        self.running = True
        self.key_type = r"{\"code.*?\"}"
        self.pattern = re.compile(r"{\"content.*?aiui_event\"}")
        self.flag_pub = flag_pub
        self.angle_pub = angle_pub

    # 麦克风阵列切换
    def switch_mic(self, mic="mic6_circle"):
        # mic：麦克风阵列类型，mic4：线性4麦，mic6：线性6麦， mic6_circle：环形6麦
        param ={
            "type": "switch_mic",
            "content": {
                "mic": "mic6_circle"
            }
        }
        param['content']['mic'] = mic 
        header = [0xA5, 0x01, 0x05]  
        res = self.send(header, param)
        if res is not None:
            pattern = re.compile(self.key_type)
            m = re.search(pattern, str(res))
            if m is not None:
                m = m.group(0)
                if m is not None:
                    return m
        
        return False

    # 获取版本信息
    def get_setting(self):
        param ={
            "type": "version"
        }
        
        header = [0xA5, 0x01, 0x05]  
        res = self.send(header, param)
        if res is not None:
            pattern = re.compile(self.key_type)
            m = re.search(pattern, str(res))
            if m is not None:
                m = m.group(0)
                if m is not None:
                    return m
        
        return False

    # 唤醒词更换（浅定制）
    def set_wakeup_word(self, str_pinyin="xiao3 huan4 xiao3 huan4"):
        # 参数为中文拼音
        # 更多参数请参考https://aiui.xfyun.cn/doc/aiui/3_access_service/access_hardware/r818/protocol.html
        param = {
            "type": "wakeup_keywords",
            "content": {
                "keyword": "xiao3 huan4 xiao3 huan4",
                "threshold": "500"
            }
        }

        param['content']['keyword'] = str_pinyin
        header = [0xA5, 0x01, 0x05]
        print('setting wakeup keywords need about 30s')
        print('setting ...')
        self.send(header, param)
        while time.time() - self.start_time < 30:
            time.sleep(0.1)

    # 计算校验和
    def calculate_checksum(self, bytes_list):
        checksum = sum(bytes_list) & 0xFF
        checksum = (~checksum + 1) & 0xFF
        return checksum

    # 数据串口发送
    def send_data(self, header, args):
        packet = header

        data = bytes(json.dumps(args), encoding="utf8")

        length = len(data)
        low_length = int(length & 0xFF)
        high_length = int(length >> 8)

        packet.extend([low_length, high_length])
        packet.extend([0x00, 0x00])

        packet.extend(data)
        checksum = self.calculate_checksum(packet)
        packet.append(checksum)

        self.serialHandle.write(packet)  # 发送主控消息

    # 发送数据
    def send(self, header, args):
        self.serialHandle.write([0xa5, 0x01, 0x01, 0x04, 0x00, 0x00, 0x00, 0xa5, 0x00, 0x00, 0x00, 0xb0])  # 发送握手请求
        while True:
            result = None
            recv_data = self.serialHandle.read()
            header_ = [b'\xa5', b'\x01', b'\xff']
            if recv_data == header_[0]:
                recv_data = self.serialHandle.read()
                if recv_data == header_[1]:
                    recv_data = self.serialHandle.read()
                    if recv_data == header_[2]:
                        recv_data = self.serialHandle.read(4)
                        self.serialHandle.read((recv_data[1] << 8 | recv_data[0]) + 1)
                        self.send_data(header, args)
                        self.start_time = time.time()
                        break

                    else:  # 没有收到确认
                        recv_data = self.serialHandle.read(4)
                        self.serialHandle.read((recv_data[1] << 8 | recv_data[0]) + 1)

                        time.sleep(0.1)
                        self.serialHandle.write(
                            [0xa5, 0x01, 0x01, 0x04, 0x00, 0x00, 0x00, 0xa5, 0x00, 0x00, 0x00, 0xb0])  # 继续发送发送握手请求

        result = None
        while True:
            recv_data = self.serialHandle.read()
            header_ = [b'\xa5', b'\x01', b'\x04']
            if recv_data == header_[0]:
                recv_data = self.serialHandle.read()
                if recv_data == header_[1]:
                    recv_data = self.serialHandle.read()
                    if recv_data == header_[2]:
                        recv_data = self.serialHandle.read(4)
                        result = self.serialHandle.read((recv_data[1] << 8 | recv_data[0]) + 1)
                        break
        return result

    def val_map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    # 检测是否唤醒以及唤醒对应角度
    def get_awake_result(self):
        while True:
            recv_data = self.serialHandle.read()
            if recv_data == b'\xa5':
                recv_data = self.serialHandle.read()
                if recv_data == b'\x01':
                    recv_data = self.serialHandle.read()
                    if recv_data == b'\x04':
                        recv_data = self.serialHandle.read(4)
                        result = self.serialHandle.read((recv_data[1] << 8 | recv_data[0]) + 1)
                        if b'content' in result:
                            m = re.search(self.pattern, str(result).replace('\\', ''))
                            if m is not None:
                                m = m.group(0).replace('"{"', '{"').replace('}"', '}')
                                if m is not None:
                                    angle = int(json.loads(m)['content']['info']['ivw']['angle'])
                                    if self.flag_pub is not None and self.angle_pub is not None:
                                        msg = Bool()
                                        msg.data = True
                                        self.flag_pub.publish(msg)
                                        angle = self.val_map(angle, 0, 360, 360, 0) + 240  # 和圆形兼容
                                        if angle >= 360:
                                            angle -= 360
                                        msg = Int32()
                                        msg.data = int(angle)
                                    self.angle_pub.publish(msg)
            time.sleep(0.02)
    
class AwakeNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.declare_parameter('mic_type', 'mic6_circle')
        self.declare_parameter('port', '/dev/ring_mic')
        # self.declare_parameter('awake_word', 'xiao3 ai4 tong2 xue3')
        self.declare_parameter('enable_setting', False)
        self.declare_parameter('awake_word', 'xiao3 huan4 xiao3 huan4')

        mic_type = self.get_parameter('mic_type').value
        port = self.get_parameter('port').value
        awake_word = self.get_parameter('awake_word').value
        enable_setting = self.get_parameter('enable_setting').value

        self.create_service(SetString, '~/set_mic_type', self.set_mic_type_srv)
        self.create_service(Trigger, '~/get_setting', self.get_setting_srv)
        self.create_service(SetString, '~/set_wakeup_word', self.set_wakeup_word_srv)

        self.awake_angle_pub = self.create_publisher(Int32, '~/angle', 1)
        self.awake_flag_pub = self.create_publisher(Bool, '~/awake_flag', 1)

        self.mic = CircleMic(port, self.awake_flag_pub, self.awake_angle_pub)
        if enable_setting:
            self.mic.switch_mic(mic_type)
            self.mic.set_wakeup_word(awake_word)
        self.get_logger().info('\033[1;32mWake up word: %s\033[0m' % awake_word)
        threading.Thread(target=self.mic.get_awake_result, daemon=True).start()

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    # 设置麦克风类型
    def set_mic_type_srv(self, resquest, response):
        res = self.mic.switch_mic(resquest.data)
        response.message = str(res)
        return response

    # 获取麦克风版本信息
    def get_setting_srv(self, request, response):
        res = self.mic.get_setting()
        response.message = str(res)
        return response

    # 设置唤醒词服务
    def set_wakeup_word_srv(self, request, response):
        self.mic.set_wakeup_word(request.data)
        return response

    def shutdown(self):
        self.mic.serialHandle.close()
        time.sleep(1)
       
def main():
    node = AwakeNode('awake_node')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('shutdown')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown() 

if __name__ == "__main__":
    main()

