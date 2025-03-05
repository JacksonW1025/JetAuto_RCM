#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 人体跟踪
import os
import cv2
import time
import math
import queue
import rclpy
import signal
import threading
import numpy as np
import faulthandler
import sdk.pid as pid
import sdk.fps as fps
import message_filters
from sdk import common
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from interfaces.msg import ObjectsInfo
from servo_controller_msgs.msg import ServosPosition
from servo_controller.action_group_controller import ActionGroupController

faulthandler.enable()

class BodyControlNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
       
        self.pid_d = pid.PID(0.1, 0, 0)
        #self.pid_d = pid.PID(0, 0, 0)
        
        self.pid_angular = pid.PID(0.002, 0, 0)
        #self.pid_angular = pid.PID(0, 0, 0)
        
        self.go_speed, self.turn_speed = 0.007, 0.04
        self.linear_x, self.angular = 0, 0
        self.running = True
        self.fps = fps.FPS()  # fps计算器
        signal.signal(signal.SIGINT, self.shutdown)
        self.image_queue = queue.Queue(maxsize=2)

        self.turn_left = False
        self.turn_right = False
        self.go_forward = False
        self.back = False
        self.next_frame = True
        self.depth_frame = None
        self.center = None
        self.machine_type = os.environ.get('MACHINE_TYPE')

        camera = 'depth_cam'
        self.create_subscription(ObjectsInfo, '/yolov5/object_detect', self.get_object_callback, 1)
        image_sub = message_filters.Subscriber(self, Image, '/yolov5/object_image')
        depth_sub = message_filters.Subscriber(self, Image, '/%s/depth/image_raw' % camera)

        # 同步时间戳, 时间允许有误差在0.03s
        sync = message_filters.ApproximateTimeSynchronizer([depth_sub, image_sub], 3, 0.02)
        sync.registerCallback(self.multi_callback)

        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # 底盘控制

        self.controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/share/arm_pc/ActionGroups')
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/yolov5/start')
        self.client.wait_for_service()

        self.future = self.client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, self.future)

        self.mecanum_pub.publish(Twist())
        self.controller.run_action('camera_up')

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False

    # 获取目标检测结果
    def get_object_callback(self, msg):
        for i in msg.objects:
            class_name = i.class_name
            if class_name == 'person':
                if i.box[1] < 10:
                    self.center = [int((i.box[0] + i.box[2])/2), int(i.box[1]) + abs(int((i.box[1] - i.box[3])/4))]
                else:
                    self.center = [int((i.box[0] + i.box[2])/2), int(i.box[1]) + abs(int((i.box[1] - i.box[3])/3))]

    def multi_callback(self, depth_image, ros_image):
        depth_frame = np.ndarray(shape=(depth_image.height, depth_image.width), dtype=np.uint16, buffer=depth_image.data)
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the custom image message into image)  into)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put([depth_frame, rgb_image])

    def image_proc(self, image):
        twist = Twist()
        depth_frame = image[0]
        bgr_image = image[1]
        if self.center is not None:
            h, w = bgr_image.shape[:-1]
            cv2.circle(bgr_image, tuple(self.center), 10, (0, 255, 255), -1) 
            #################
            roi_h, roi_w = 5, 5
            w_1 = self.center[0] - roi_w
            w_2 = self.center[0] + roi_w
            if w_1 < 0:
                w_1 = 0
            if w_2 > w:
                w_2 = w
            h_1 = self.center[1] - roi_h
            h_2 = self.center[1] + roi_h
            if h_1 < 0:
                h_1 = 0
            if h_2 > h:
                h_2 = h
            
            cv2.rectangle(bgr_image, (w_1, h_1), (w_2, h_2), (0, 255, 255), 2)
            roi = depth_frame[h_1:h_2, w_1:w_2]
            distances = roi[np.logical_and(roi > 0, roi < 40000)]
            if len(distances) > 0:
                distance = int(np.mean(distances)/10)
            else:
                distance = 0
                #print(distance)
            ################
            if distance > 600: 
                distance = 600
            elif distance < 60:
                distance = 60
            
            if abs(distance - 150) < 15:
                distance = 150
            self.pid_d.update(distance - 150)  # 更新pid(update pid)
            self.linear_x = -common.set_range(self.pid_d.output, -0.4, 0.4)
            
            if abs(self.center[0] - w/2) < 30:
                self.center[0] = w/2
            self.pid_angular.update(self.center[0] - w/2)  # 更新pid(update pid)
            twist.linear.x = self.linear_x
            twist.angular.z = common.set_range(self.pid_angular.output, -0.8, 0.8)
            self.mecanum_pub.publish(twist)
        self.mecanum_pub.publish(twist)
        return bgr_image

    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            try:
                result_image = self.image_proc(image)
            except BaseException as e:
                self.get_logger().info('\033[1;32m%s\033[0m' % e)
            self.center = None
            # cv2.imshow(self.name, result_image)
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 按q或者esc退出
                self.mecanum_pub.publish(Twist())
                self.running = False

        rclpy.shutdown()

def main():
    node = BodyControlNode('body_control')
    rclpy.spin(node)
    node.destroy_node()

if __name__ == "__main__":
    main()
