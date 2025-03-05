#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 垃圾分类
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
from geometry_msgs.msg import Twist
from interfaces.msg import ObjectsInfo
from xf_mic_asr_offline import voice_play
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from servo_controller.action_group_controller import ActionGroupController

WASTE_CLASSES = {
    'food_waste': ('BananaPeel', 'BrokenBones', 'Ketchup'),
    'hazardous_waste': ('Marker', 'OralLiquidBottle', 'StorageBattery'),
    'recyclable_waste': ('PlasticBottle', 'Toothbrush', 'Umbrella'),
    'residual_waste': ('Plate', 'CigaretteEnd', 'DisposableChopsticks'),
}

class GarbageClassificationNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.running = True
        self.center = None
        self.count = 0
        self.class_name = None
        self.start_pick = False
        self.current_class_name = None
        self.language = os.environ['ASR_LANGUAGE']
        pick_roi = self.get_parameters_by_prefix('roi')
        self.pick_roi = [pick_roi['y_min'].value, pick_roi['y_max'].value, pick_roi['x_min'].value, pick_roi['x_max'].value] #[y_min, y_max, x_min, x_max]

        signal.signal(signal.SIGINT, self.shutdown)
        self.image_queue = queue.Queue(maxsize=2)
        self.bridge = CvBridge()
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)

        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback, callback_group=timer_cb_group)  # 进入玩法
        self.create_service(Trigger, '~/stop', self.stop_srv_callback, callback_group=timer_cb_group)  # 退出玩法
        self.create_subscription(Image, '/yolov5/object_image', self.image_callback, 1)
        self.create_subscription(ObjectsInfo, '/yolov5/object_detect', self.get_object_callback, 1)
        self.start_yolov5_client = self.create_client(Trigger, '/yolov5/start', callback_group=timer_cb_group)
        self.start_yolov5_client.wait_for_service()
        self.stop_yolov5_client = self.create_client(Trigger, '/yolov5/stop', callback_group=timer_cb_group)
        self.stop_yolov5_client.wait_for_service()

        self.debug = self.get_parameter('debug').value
        self.broadcast = self.get_parameter('broadcast').value

        self.controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/share/arm_pc/ActionGroups')
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()

        self.mecanum_pub.publish(Twist())
        self.controller.run_action('garbage_pick_init')
        if self.debug:
            self.pick_roi = [30, 450, 30, 610]
            self.controller.run_action('garbage_pick_debug')
            time.sleep(5)
            self.controller.run_action('garbage_pick_init')
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

    def play(self, name):
        if self.broadcast:
            voice_play.play(name, language=self.language)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start garbage classification")

        self.send_request(self.start_yolov5_client, Trigger.Request())
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop garbage classification")
        self.send_request(self.stop_yolov5_client, Trigger.Request())
        response.success = True
        response.message = "stop"
        return response
    def shutdown(self, signum, frame):
        self.running = False

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(rgb_image)

    def pick(self):
        while self.running:
            waste_category = None
            if self.start_pick:
                time.sleep(0.2)
                for k, v in WASTE_CLASSES.items():
                    if self.current_class_name in v:
                        waste_category = k
                        break
                self.class_name = None
                self.get_logger().info('\033[1;32m%s\033[0m' % waste_category)
                self.stop_srv_callback(Trigger.Request(), Trigger.Response())
                self.controller.run_action('garbage_pick')
                if waste_category == 'food_waste':
                    self.play('food_waste')
                    self.controller.run_action('place_food_waste')
                elif waste_category == 'hazardous_waste':
                    self.play('hazardous_waste')
                    self.controller.run_action('place_hazardous_waste')
                elif waste_category == 'recyclable_waste':
                    self.play('recyclable_waste')
                    self.controller.run_action('place_recyclable_waste')
                elif waste_category == 'residual_waste':
                    self.play('residual_waste')
                    self.controller.run_action('place_residual_waste')
                self.controller.run_action('garbage_pick_init')
                time.sleep(0.5)
                self.start_pick = False
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
            if self.class_name is not None and not self.start_pick and not self.debug:
                self.count += 1
                if self.count > 50:
                    self.current_class_name = self.class_name
                    self.start_pick = True
                    self.count = 0
            elif self.debug and self.class_name is not None:
                count += 1
                if count > 50:
                    count = 0
                    self.pick_roi = [self.center[1] - 15, self.center[1] + 15, self.center[0] - 15, self.center[0] + 15]
                    data = {'/**': {'ros__parameters': {'roi': {}}}}
                    roi = data['/**']['ros__parameters']['roi']
                    roi['x_min'] = self.pick_roi[2]
                    roi['x_max'] = self.pick_roi[3]
                    roi['y_min'] = self.pick_roi[0]
                    roi['y_max'] = self.pick_roi[1]
                    common.save_yaml_data(data, os.path.join(
                        os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '../..')),
                        'config/garbage_classification_roi.yaml'))
                    self.debug = False
                self.get_logger().info(str([self.center[1] - 15, self.center[1] + 15, self.center[0] - 15, self.center[0] + 15]))
                cv2.rectangle(image, (self.center[0] - 45, self.center[1] - 45), (self.center[0] + 45, self.center[1] + 45), (0, 0, 255), 2)
            else:
                self.count = 0
                time.sleep(0.01)
            if image is not None:
                if not self.start_pick and not self.debug:
                    cv2.rectangle(image, (self.pick_roi[2] - 30, self.pick_roi[0] - 30), (self.pick_roi[3] + 30, self.pick_roi[1] + 30), (0, 255, 255), 2)
                cv2.imshow('image', image)
                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:  # 按q或者esc退出
                    self.running = False
        self.mecanum_pub.publish(Twist())
        self.controller.run_action('garbage_pick_init')

    def get_object_callback(self, msg):
        objects = msg.objects
        if objects == []:
            self.center = None
            self.class_name = None
        else:
            for i in objects:
                center = (int((i.box[0] + i.box[2])/2), int((i.box[1] + i.box[3])/2))
                if self.pick_roi[2] < center[0] < self.pick_roi[3] and self.pick_roi[0] < center[1] < self.pick_roi[1]:
                    self.center = center
                    self.class_name = i.class_name


def main():
    node = GarbageClassificationNode('garbage_classification')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

