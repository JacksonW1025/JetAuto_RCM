#!/usr/bin/env python3
# encoding: utf-8
# @data:2024/02/28
# @author:aiden
# 巡线清障
import os
import cv2
import time
import math
import rclpy
import queue
import signal
import threading
import numpy as np
import sdk.pid as pid
from sdk import common
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from interfaces.msg import ColorsInfo, ColorDetect, LineROI, ROI
from interfaces.srv import SetColorDetectParam, SetCircleROI, SetLineROI
from servo_controller.action_group_controller import ActionGroupController

MAX_SCAN_ANGLE = 240  # 激光的扫描角度,去掉总是被遮挡的部分degree

class LineFollowCleanNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.running = True
        self.count = 0
        self.count_stop = 0
        self.stop = False
        self.line_color = 'black'
        self.line_x = None
        self.temp_line_x = None
        self.object_blue = 'blue' 
        self.object_red = 'red'
        self.object_green = 'green'
        self.center = None
        self.temp_center = None
        self.stop_threshold = 0.4
        self.scan_angle = math.radians(90)  # radians
        self.pid = pid.PID(0.008, 0.0, 0.0)
        self.pid_x = pid.PID(0.001, 0.0, 0.0)
        pick_roi = self.get_parameters_by_prefix('roi')
        self.pick_roi = [pick_roi['y_min'].value, pick_roi['y_max'].value, pick_roi['x_min'].value, pick_roi['x_max'].value] #[y_min, y_max, x_min, x_max]
        self.start_pick = False
        
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        signal.signal(signal.SIGINT, self.shutdown)
        self.lidar_type = os.environ.get('LIDAR_TYPE')

        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.create_subscription(ColorsInfo, '/color_detect/color_info', self.get_color_callback, 1)
        self.create_subscription(Image, '/color_detect/image_result', self.image_callback, 1)

        self.controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 0), '/home/ubuntu/share/arm_pc/ActionGroups')
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback, callback_group=timer_cb_group) # 进入玩法
        self.create_service(Trigger, '~/stop', self.stop_srv_callback, callback_group=timer_cb_group) # 退出玩法
        self.set_line_client = self.create_client(SetLineROI, '/color_detect/set_line_roi', callback_group=timer_cb_group)
        self.set_line_client.wait_for_service()
        self.set_circle_client = self.create_client(SetCircleROI, '/color_detect/set_circle_roi', callback_group=timer_cb_group)
        self.set_circle_client.wait_for_service()
        self.set_color_client = self.create_client(SetColorDetectParam, '/color_detect/set_param', callback_group=timer_cb_group)
        self.set_color_client.wait_for_service()

        self.debug = self.get_parameter('debug').value

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()

        self.mecanum_pub.publish(Twist())
        self.controller.run_action('line_follow_init')
        if self.debug:
            self.controller.run_action('move_object_debug')
            time.sleep(5)
            self.controller.run_action('line_follow_init')
            time.sleep(2)

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
        self.get_logger().info('\033[1;32m%s\033[0m' % "start line follow clean")

        line_roi = LineROI()
        line_roi.roi_up.x_min = 0
        line_roi.roi_up.x_max = 640
        line_roi.roi_up.y_min = 200
        line_roi.roi_up.y_max = 210
        line_roi.roi_up.scale = 0.0

        line_roi.roi_center.x_min = 0
        line_roi.roi_center.x_max = 640
        line_roi.roi_center.y_min = 260
        line_roi.roi_center.y_max = 270
        line_roi.roi_center.scale = 0.1

        line_roi.roi_down.x_min = 0
        line_roi.roi_down.x_max = 640
        line_roi.roi_down.y_min = 320
        line_roi.roi_down.y_max = 330
        line_roi.roi_down.scale = 0.9
        msg = SetLineROI.Request()
        msg.data = line_roi
        res = self.send_request(self.set_line_client, msg)
        if res.success:
            self.get_logger().info('set roi success')
        else:
            self.get_logger().info('set roi fail')

        object_roi = ROI()
        object_roi.x_min = 0
        object_roi.x_max = 640
        object_roi.y_min = 0
        object_roi.y_max = 300
        msg = SetCircleROI.Request()
        msg.data = object_roi
        res = self.send_request(self.set_circle_client, msg)
        if res.success:
            self.get_logger().info('set roi success')
        else:
            self.get_logger().info('set roi fail')
        
        msg_black = ColorDetect()
        msg_black.color_name = self.line_color
        msg_black.detect_type = 'line'
        msg_blue = ColorDetect()
        msg_blue.color_name = self.object_blue
        msg_blue.detect_type = 'circle'
        msg_red = ColorDetect()
        msg_red.color_name = self.object_red
        msg_red.detect_type = 'circle'
        msg_green = ColorDetect()
        msg_green.color_name = self.object_green
        msg_green.detect_type = 'circle'
        msg = SetColorDetectParam.Request()
        msg.data = [msg_red, msg_green, msg_blue, msg_black]
        res = self.send_request(self.set_color_client, msg)
        if res.success:
            self.get_logger().info('set color success')
        else:
            self.get_logger().info('set color fail')

        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop line follow clean")
        res = self.send_request(self.set_color_client, SetColorDetectParam.Request())
        if res.success:
            self.get_logger().info('set color success')
        else:
            self.get_logger().info('set color fail')

        response.success = True
        response.message = "stop"
        return response

    def get_color_callback(self, msg):
        line_x = None
        center = None
        for i in msg.data:
            if i.color == self.line_color:
                line_x = i.x
            elif i.color == self.object_blue or i.color == self.object_red or i.color == self.object_green:
                center = i
        self.temp_line_x = line_x
        self.temp_center = center

    def pick(self):
        while self.running:
            if self.start_pick:
                self.stop_srv_callback(Trigger.Request(), Trigger.Response())
                self.mecanum_pub.publish(Twist())
                time.sleep(0.5)
                self.controller.run_action('move_object')
                self.controller.run_action('line_follow_init')
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
            self.line_x = self.temp_line_x
            self.center = self.temp_center
            if self.line_x is not None and not self.start_pick:
                twist = Twist()
                if self.center is not None:
                    if self.center.y > 100 and abs(self.center.x - self.line_x) < 100 and not self.debug:
                        self.pid_x.SetPoint = (self.pick_roi[1] + self.pick_roi[0])/2
                        self.pid_x.update(self.center.y)
                        self.pid.SetPoint = (self.pick_roi[2] + self.pick_roi[3])/2
                        self.pid.update(self.center.x)
                        twist.linear.x = common.set_range(self.pid_x.output, -0.1, 0.1)
                        twist.angular.z = common.set_range(self.pid.output, -0.5, 0.5)
                        if abs(twist.linear.x) <= 0.0065 and abs(twist.angular.z) <= 0.05:
                            self.count += 1
                            time.sleep(0.01)
                            if self.count > 50:
                                self.count = 0
                                self.start_pick = True
                        else:
                            self.count = 0
                    elif self.debug:
                        count += 1
                        if count > 50:
                            count = 0
                            self.pick_roi = [self.center.y - 15, self.center.y + 15, self.center.x - 15, self.center.x + 15]
                            data = {'/**': {'ros__parameters': {'roi': {}}}}
                            roi = data['/**']['ros__parameters']['roi']
                            roi['x_min'] = self.pick_roi[2]
                            roi['x_max'] = self.pick_roi[3]
                            roi['y_min'] = self.pick_roi[0]
                            roi['y_max'] = self.pick_roi[1]
                            common.save_yaml_data(data, os.path.join(
                                os.path.abspath(os.path.join(os.path.split(os.path.realpath(__file__))[0], '../..')),
                                'config/line_follow_clean_roi.yaml'))
                            self.debug = False
                            self.start_srv_callback(Trigger.Request(), Trigger.Response())
                        self.get_logger().info(str([self.center.y - 15, self.center.y + 15, self.center.x - 15, self.center.x + 15]))
                        cv2.rectangle(image, (self.center.x - 25, self.center.y - 25,), (self.center.x + 25, self.center.y + 25), (0, 0, 255), 2)
                    else:
                        self.pid.SetPoint = 320
                        self.pid.update(self.line_x)
                        twist.linear.x = 0.08
                        twist.angular.z = common.set_range(self.pid.output, -0.8, 0.8)
                elif not self.debug:
                    self.pid.SetPoint = 320
                    self.pid.update(self.line_x)
                    twist.linear.x = 0.15
                    twist.angular.z = common.set_range(self.pid.output, -0.8, 0.8)
                if not self.stop:
                    self.mecanum_pub.publish(twist)
                else:
                    self.mecanum_pub.publish(Twist())
            else:
                self.mecanum_pub.publish(Twist())
                time.sleep(0.01)
            if image is not None:
                if not self.start_pick and not self.debug:
                    cv2.rectangle(image, (self.pick_roi[2] - 30, self.pick_roi[0] - 30), (self.pick_roi[3] + 30, self.pick_roi[1] + 30), (0, 255, 255), 2)
                cv2.imshow('image', image)
                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:  # 按q或者esc退出
                    self.running = False
        self.mecanum_pub.publish(Twist())
        self.controller.run_action('line_follow_init')
        rclpy.shutdown()

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        cv_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(cv_image)

    def lidar_callback(self, lidar_data):
        # 数据大小 = 扫描角度/每扫描一次增加的角度
        if self.lidar_type != 'G4':
            max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[:max_index]  # 左半边数据
            right_ranges = lidar_data.ranges[::-1][:max_index]  # 右半边数据
        elif self.lidar_type == 'G4':
            min_index = int(math.radians((360 - MAX_SCAN_ANGLE) / 2.0) / lidar_data.angle_increment)
            max_index = int(math.radians(180) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[::-1][min_index:max_index][::-1] # 左半边数据
            right_ranges = lidar_data.ranges[min_index:max_index][::-1] # 右半边数据

        # 根据设定取数据
        angle = self.scan_angle / 2
        angle_index = int(angle / lidar_data.angle_increment + 0.50)
        left_range, right_range = np.array(left_ranges[:angle_index]), np.array(right_ranges[:angle_index])

        left_nonzero = left_range.nonzero()
        right_nonzero = right_range.nonzero()
        # 取左右最近的距离
        min_dist_left = left_range[left_nonzero].min()
        min_dist_right = right_range[right_nonzero].min()
        if min_dist_left < self.stop_threshold or min_dist_right < self.stop_threshold: 
            self.stop = True
        else:
            self.count_stop += 1
            if self.count_stop > 5:
                self.count_stop = 0
                self.stop = False


def main():
    node = LineFollowCleanNode('line_follow_clean')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
