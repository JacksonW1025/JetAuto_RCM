#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/19
# @author:aiden
# 手势控制
import cv2
import math
import time
import rclpy
import signal
import threading
import numpy as np
from rclpy.node import Node
from std_srvs.srv import Trigger
from interfaces.msg import Points
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position
from servo_controller.action_group_controller import ActionGroupController

class HandGestureControlNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.image = None
        self.points = []
        self.running = True
        self.left_and_right = 0
        self.up_and_down = 0
        self.last_point = [0, 0]

        signal.signal(signal.SIGINT, self.shutdown)
        self.create_subscription(Points, '/hand_trajectory/points', self.get_hand_points_callback, 1)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)  # 舵机控制
        self.controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/share/arm_pc/ActionGroups')
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

        set_servo_position(self.joints_pub, 1.5, ((10, 300), (5, 500), (4, 600), (3, 0), (2, 750), (1, 500)))
        time.sleep(1.5)

        threading.Thread(target=self.hand_gesture_control, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False

    def get_hand_points_callback(self, msg):
        points = []
        left_and_right = [0]
        up_and_down = [0]
        if len(msg.points) > 5:
            for i in msg.points:
                if int(i.x) - self.last_point[0] > 0:
                    left_and_right.append(1)
                else:
                    left_and_right.append(-1)
                if int(i.y) - self.last_point[1] > 0:
                    up_and_down.append(1)
                else:
                    up_and_down.append(-1)
                points.extend([(int(i.x), int(i.y))])
                self.last_point = [int(i.x), int(i.y)]
            self.left_and_right = sum(left_and_right)
            self.up_and_down = sum(up_and_down)
            self.points = np.array(points)

    def hand_gesture_control(self):
        while self.running:
            if len(self.points):
                line = cv2.fitLine(self.points, cv2.DIST_L2, 0, 0.01, 0.01)
                angle = int(abs(math.degrees(math.acos(line[0][0]))))
                self.get_logger().info('>>>>>>%s'%angle)
                if 90 >= angle > 60:
                    if self.up_and_down > 0:
                        self.get_logger().info('down')
                    else:
                        self.get_logger().info('up')
                    time.sleep(0.3)
                    self.controller.run_action('hand_control_pick')
                    set_servo_position(self.joints_pub, 1.5, ((5, 500), (4, 600), (3, 0), (2, 750), (1, 500)))
                elif 30 > angle >= 0:
                    if self.left_and_right > 0:
                        self.get_logger().info('right')
                    else:
                        self.get_logger().info('left')
                    time.sleep(0.3)
                    self.controller.run_action('hand_control_place')
                    set_servo_position(self.joints_pub, 1.5, ((5, 500), (4, 600), (3, 0), (2, 750), (1, 500)))
                self.points = []
            else:
                time.sleep(0.01)

        self.controller.run_action('horizontal')
        rclpy.shutdown()

def main():
    node = HandGestureControlNode('hand_gesture_control')
    rclpy.spin(node)
    node.destroy_node()

if __name__ == "__main__":
    main()
