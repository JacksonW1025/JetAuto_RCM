#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2023/11/10
from servo_controller_msgs.msg import ServoPosition, ServosPosition

def set_servo_position(pub, duration, positions):
    msg = ServosPosition()
    msg.duration = float(duration)
    position_list = []
    for i in positions:
        position = ServoPosition()
        position.id = i[0]
        position.position = i[1]
        position_list.append(position)
    msg.position = position_list
    pub.publish(msg)

if __name__ == '__main__':
    import time
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node('servo_control_demo')
    pub = node.create_publisher(ServosPosition, 'servo_controller', 1)

    while rclpy.ok():
        set_servo_position(pub, 0.02, ((10, 500), (5, 500), (4, 500), (3, 500), (2, 500), (1, 500)))
        time.sleep(0.02)

        set_servo_position(pub, 0.02, ((10, 500), (5, 500), (4, 500), (3, 500), (2, 500), (1, 500)))
        time.sleep(0.02)
