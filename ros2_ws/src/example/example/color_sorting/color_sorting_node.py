#!/usr/bin/env python3
# encoding: utf-8
# @data:2024/02/28
# @author:aiden
# 颜色分拣
import os
import cv2
import time
import queue
import rclpy
import signal
import threading
import numpy as np
from sdk import common
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from interfaces.msg import ColorsInfo, ColorDetect, ROI
from rclpy.callback_groups import ReentrantCallbackGroup
from interfaces.srv import SetColorDetectParam, SetCircleROI
from servo_controller.action_group_controller import ActionGroupController

class ColorSortingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.running = True
        self.center = None
        pick_roi = self.get_parameters_by_prefix('roi')
        #self.pick_roi = [290, 310, 310, 330] #[y_min, y_max, x_min, x_max]
        self.pick_roi = [pick_roi['y_min'].value, pick_roi['y_max'].value, pick_roi['x_min'].value, pick_roi['x_max'].value] #[y_min, y_max, x_min, x_max]
        self.start = False
        self.start_pick = False
        self.target_color = ''
        self.color = ''
        self.count = 0
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        signal.signal(signal.SIGINT, self.shutdown)
        
        self.create_subscription(ColorsInfo, '/color_detect/color_info', self.get_color_callback, 1)
        self.create_subscription(Image, '/color_detect/image_result', self.image_callback, 1)
        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback, callback_group=timer_cb_group) # 进入玩法
        self.create_service(Trigger, '~/stop', self.stop_srv_callback, callback_group=timer_cb_group) # 退出玩法

        self.controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/share/arm_pc/ActionGroups')
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.controller.run_action('pick_init')

        self.set_color_client = self.create_client(SetColorDetectParam, '/color_detect/set_param', callback_group=timer_cb_group)
        self.set_roi_client = self.create_client(SetCircleROI, '/color_detect/set_circle_roi', callback_group=timer_cb_group)
        self.set_color_client.wait_for_service()
        self.set_roi_client.wait_for_service()
        self.debug = self.get_parameter('debug').value

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()

        if self.debug:
            self.pick_roi = [20, 340, 20, 620]
            self.controller.run_action('pick_debug')
            time.sleep(5)
            self.controller.run_action('pick_init')
            time.sleep(2)
        if self.get_parameter('start').value:
            self.start_srv_callback(Trigger.Request(), Trigger.Response())

        threading.Thread(target=self.pick, daemon=True).start()
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start color sorting")
        roi = ROI()
        roi.x_min = self.pick_roi[2] - 20
        roi.x_max = self.pick_roi[3] + 20
        roi.y_min = self.pick_roi[0] - 20
        roi.y_max = self.pick_roi[1] + 20
        msg = SetCircleROI.Request()
        msg.data = roi

        res = self.send_request(self.set_roi_client, msg)
        if res.success:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'set roi success')
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'set roi fail')
        
        msg = SetColorDetectParam.Request()
        msg_red = ColorDetect()
        msg_red.color_name = 'red'
        msg_red.detect_type = 'circle'
        msg_green = ColorDetect()
        msg_green.color_name = 'green'
        msg_green.detect_type = 'circle'
        msg_blue = ColorDetect()
        msg_blue.color_name = 'blue'
        msg_blue.detect_type = 'circle'
        msg.data = [msg_red, msg_green, msg_blue]
        res = self.send_request(self.set_color_client, msg)
        if res.success:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'set color success')
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'set color fail')
        self.start = True
         
        response.success = True
        response.message = "start"
        return response
    
    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop color sorting")
        self.start = False
        res = self.send_request(self.set_color_client, SetColorDetectParam.Request())
        if res.success:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'set color success')
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'set color fail')

        response.success = True
        response.message = "stop"
        return response   

    def get_color_callback(self, msg):
        data = msg.data
        if data != []:
            if data[0].radius > 10:
                self.center = data[0]
                self.color = data[0].color
            else:
                self.color = ''
        else:
            self.color = ''

    def pick(self):
        while self.running:
            if self.start_pick:
                self.stop_srv_callback(Trigger.Request(), Trigger.Response())
                self.get_logger().info('\033[1;32mcolor: %s\033[0m' % self.target_color)
                if self.target_color == 'red':
                    self.controller.run_action('pick')
                    self.controller.run_action('place_center')
                elif self.target_color == 'green':
                    self.controller.run_action('pick')
                    self.controller.run_action('place_left')
                elif self.target_color == 'blue':
                    self.controller.run_action('pick')
                    self.controller.run_action('place_right')
                self.start_pick = False
                self.controller.run_action('pick_init')
                time.sleep(0.5)
                self.start_srv_callback(Trigger.Request(), Trigger.Response())
            else:
                time.sleep(0.01)

    def main(self):
        count = 0
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            if self.color in ['red', 'green', 'blue'] and self.start:
                if self.pick_roi[2] < self.center.x < self.pick_roi[3] and self.pick_roi[0] < self.center.y < self.pick_roi[1] and not self.start_pick and not self.debug:
                    self.count += 1
                    if self.count > 30:
                        self.count = 0
                        self.target_color = self.color
                        self.start_pick = True
                elif self.debug:
                    count += 1
                    if count > 50:
                        count = 0
                        self.pick_roi = [self.center.y - 10, self.center.y + 10, self.center.x - 10, self.center.x + 10]
                        data = {'/**': {'ros__parameters': {'roi': {}}}}
                        roi = data['/**']['ros__parameters']['roi']
                        roi['x_min'] = self.pick_roi[2]
                        roi['x_max'] = self.pick_roi[3]
                        roi['y_min'] = self.pick_roi[0]
                        roi['y_max'] = self.pick_roi[1]
                        common.save_yaml_data(data, os.path.join(
                            os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '../..')),
                            'config/color_sorting_roi.yaml'))
                        self.start_srv_callback(Trigger.Request(), Trigger.Response())
                        self.debug = False
                    self.get_logger().info(str([self.center.y - 10, self.center.y + 10, self.center.x - 10, self.center.x + 10]))
                    cv2.rectangle(image, (self.center.x - 25, self.center.y - 25,), (self.center.x + 25, self.center.y + 25), (0, 0, 255), 2)
                else:
                    count = 0
            if image is not None:
                if not self.start_pick and not self.debug:
                    cv2.rectangle(image, (self.pick_roi[2] - 25, self.pick_roi[0] - 25), (self.pick_roi[3] + 25, self.pick_roi[1] + 25), (0, 255, 255), 2)
                cv2.imshow('image', image)
                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:  # 按q或者esc退出
                    self.running = False
        self.controller.run_action('init')
        rclpy.shutdown()

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
            # 将图像放入队列
        self.image_queue.put(rgb_image)

def main():
    node = ColorSortingNode('color_sorting')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()

