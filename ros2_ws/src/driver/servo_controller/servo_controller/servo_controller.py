#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2023/11/10
from rclpy.node import Node
from ros_robot_controller_msgs.srv import GetBusServoState
from ros_robot_controller_msgs.msg import ServoPosition, ServosPosition

class ServoState:
    def __init__(self, name=''):
        self.name = name
        self.position = 500

class ServoManager(Node):
    def __init__(self, connected_ids=[]):
        super().__init__('servo_manager')
        self.servos = {}
        self.connected_ids = connected_ids
        for i in connected_ids:
            self.servos[i] = ServoState(connected_ids[i])
        self.servo_position_pub = self.create_publisher(ServosPosition, 'ros_robot_controller/bus_servo/set_position', 1)
        self.client = self.create_client(GetBusServoState, 'ros_robot_controller/bus_servo/get_state')
        self.client.wait_for_service()
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def connect(self):
        # 通过读取id来检测舵机
        pass
        # if self.servos:
            # for servo_id in self.servos:
                # if not self.get_servo_id(servo_id):
                    # self.get_logger().error('ID %d not found.'%servo_id)
                    # # sys.exit(1)
        # else:
            # self.get_logger().error('Motors not found.')
            # # sys.exit(1)
        # status_str = 'Found {} motors - {}'.format(len(self.servos), self.connected_ids)
        # self.get_logger().info('%s, initialization complete.' % status_str[:-2])

    def get_position(self):
        return self.servos

    def get_servo_id(self, servo_id):
        request = GetBusServoStateRequest()
        request.id = servo_id
        request.get_id = 1
        for i in range(0, 20):
            response = self.client.call(request)
            if response[0].present_id == servo_id:
                return True
        return False

    def set_position(self, duration, position):
        duration = 0.02 if duration < 0.02 else 30 if duration > 30 else duration
        msg = ServosPosition()
        msg.duration = float(duration)
        for i in position:
            position = int(i.position)
            position = 0 if position < 0 else 1000 if position > 1000 else position
            self.servos[str(i.id)].position = position  # 记录发送的位置
            servo_msg = ServoPosition()
            servo_msg.id = i.id
            servo_msg.position = position
            msg.position.append(servo_msg)
        self.servo_position_pub.publish(msg)
