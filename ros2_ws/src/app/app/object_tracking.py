#!/usr/bin/env python3
# encoding: utf-8
# 颜色跟踪
import os
import cv2
import math
import queue
import rclpy
import threading
import numpy as np
import sdk.pid as pid
import sdk.common as common
from rclpy.node import Node
from sensor_msgs.msg import Image
from app.common import ColorPicker
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger
from interfaces.srv import SetPoint, SetFloat64
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position

class ObjectTracker:
    def __init__(self, color, node):
        self.node = node
        self.pid_yaw = pid.PID(0.006, 0.0, 0.0)
        self.pid_dist = pid.PID(0.002, 0.0, 0.00)
        self.last_color_circle = None
        self.lost_target_count = 0
        self.target_lab, self.target_rgb = color
        self.weight_sum = 1.0
        self.y_stop = 380
        self.x_stop = 320
        self.pro_size = (320, 180)

    def __call__(self, image, result_image, threshold):
        twist = Twist()
        h, w = image.shape[:2]
        image = cv2.resize(image, self.pro_size)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)  # RGB转LAB空间
        image = cv2.GaussianBlur(image, (5, 5), 5)

        min_color = [int(self.target_lab[0] - 50 * threshold * 2),
                     int(self.target_lab[1] - 50 * threshold),
                     int(self.target_lab[2] - 50 * threshold)]
        max_color = [int(self.target_lab[0] + 50 * threshold * 2),
                     int(self.target_lab[1] + 50 * threshold),
                     int(self.target_lab[2] + 50 * threshold)]
        target_color = self.target_lab, min_color, max_color
        mask = cv2.inRange(image, tuple(target_color[1]), tuple(target_color[2]))  # 二值化
        # cv2.imshow('mask', cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR))
        # cv2.waitKey(1)
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        contour_area = map(lambda c: (c, math.fabs(cv2.contourArea(c))), contours)  # 计算各个轮廓的面积
        contour_area = list(filter(lambda c: c[1] > 40, contour_area))  # 剔除>面积过小的轮廓
        circle = None
        if len(contour_area) > 0:
            if self.last_color_circle is None:
                contour, area = max(contour_area, key=lambda c_a: c_a[1])
                circle = cv2.minEnclosingCircle(contour)
            else:
                (last_x, last_y), last_r = self.last_color_circle
                circles = map(lambda c: cv2.minEnclosingCircle(c[0]), contour_area)
                circle_dist = list(map(lambda c: (c, math.sqrt(((c[0][0] - last_x) ** 2) + ((c[0][1] - last_y) ** 2))),
                                       circles))
                circle, dist = min(circle_dist, key=lambda c: c[1])
                if dist < 100:
                    circle = circle
        if circle is not None:
            self.lost_target_count = 0
            (x, y), r = circle
            x = x / self.pro_size[0] * w
            y = y / self.pro_size[1] * h
            r = r / self.pro_size[0] * w

            cv2.circle(result_image, (self.x_stop, self.y_stop), 5, (255, 255, 0), -1)
            result_image = cv2.circle(result_image, (int(x), int(y)), int(r), (self.target_rgb[0],
                                                                               self.target_rgb[1],
                                                                               self.target_rgb[2]), 2)
            vx = 0
            vw = 0
            if abs(y - self.y_stop) > 20:
                self.pid_dist.update(y - self.y_stop)
                twist.linear.x = common.set_range(self.pid_dist.output, -0.35, 0.35)
            else:
                self.pid_dist.clear()
            if abs(x - self.x_stop) > 20:
                self.pid_yaw.update(x - self.x_stop)
                twist.angular.z = common.set_range(self.pid_yaw.output, -2, 2)
            else:
                self.pid_yaw.clear()

        return result_image, twist

class OjbectTrackingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.name = name
        self.set_callback = False
        self.color_picker = None
        self.tracker = None
        self.is_running = False
        self.threshold = 0.1
        self.dist_threshold = 0.3
        self.lock = threading.RLock()
        self.image_sub = None
        self.result_image = None
        self.image_height = None
        self.image_width = None
        self.image_queue = queue.Queue(2)
        self.machine_type = os.environ.get('MACHINE_TYPE')
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result',  1)
        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.set_running_srv = self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)
        self.set_target_color_srv = self.create_service(SetPoint, '~/set_target_color', self.set_target_color_srv_callback)
        self.get_target_color_srv = self.create_service(Trigger, '~/get_target_color', self.get_target_color_srv_callback)
        self.set_threshold_srv = self.create_service(SetFloat64, '~/set_threshold', self.set_threshold_srv_callback)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        # self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(None))

        self.debug = True
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def main(self):
        while True:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            result = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imshow("result", result)
            if self.debug and not self.set_callback:
                self.set_callback = True
                # 设置鼠标点击事件的回调函数
                cv2.setMouseCallback("result", self.mouse_callback)
            k = cv2.waitKey(1)
            if k != -1:
                break

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.get_logger().info("x:{} y{}".format(x, y))
            msg = SetPoint.Request()
            if self.image_height is not None and self.image_width is not None:
                msg.data.x = x / self.image_width
                msg.data.y = y / self.image_height
                self.set_target_color_srv_callback(msg, SetPoint.Response())

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'object tracking enter')
        with self.lock:
            try:
                if self.image_sub is not None:
                    self.image_sub.unregister()
            except Exception as e:
                self.get_logger().error(str(e))
            self.is_running = False
            self.threshold = 0.5
            self.tracker = None
            self.color_picker = None
            self.dist_threshold = 0.3
            if self.machine_type == 'JetAuto':
                self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
            else:
                self.image_sub = self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
            set_servo_position(self.joints_pub, 1, ((10, 300), (5, 500), (4, 210), (3, 40), (2, 750), (1, 500)))
            self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'object tracking exit')
        with self.lock:
            try:
                if self.image_sub is not None:
                    self.image_sub.unregister()
            except Exception as e:
                self.get_logger().error(str(e))
            self.is_running = False
            self.color_picker = None
            self.tracker = None
            self.threshold = 0.5
            self.dist_threshold = 0.3
            self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "exit"
        return response

    def set_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set_target_color')
        with self.lock:
            x, y = request.data.x, request.data.y
            if x == -1 and y == -1:
                self.color_picker = None
                self.tracker = None
            else:
                self.tracker = None
                self.color_picker = ColorPicker(request.data, 20)
            self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "set_target_color"
        return response

    def get_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'get_target_color')
        response.success = False
        response.message = "get_target_color"
        with self.lock:
            if self.tracker is not None:
                response.success = True
                rgb = self.tracker.target_rgb
                response.message = "{},{},{}".format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
        return response

    def set_running_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set_running')
        with self.lock:
            self.is_running = request.data
            if not self.is_running:
                self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "set_running"
        return response

    def set_threshold_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'threshold')
        with self.lock:
            self.threshold = request.data
            response.success = True
            response.message = "set_threshold"
            return response

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 原始 RGB 画面
        self.image_height = ros_image.height
        self.image_width = ros_image.width
        result_image = np.copy(rgb_image)  # 显示结果用的画面
        with self.lock:
            # 颜色拾取器和识别追踪互斥, 如果拾取器存在就开始拾取
            if self.color_picker is not None:  # 拾取器存在
                target_color, result_image = self.color_picker(rgb_image, result_image)
                if target_color is not None:
                    self.color_picker = None
                    self.tracker = ObjectTracker(target_color, self)
            else:
                if self.tracker is not None:
                    try:
                        result_image, twist = self.tracker(rgb_image, result_image, self.threshold)
                        if self.is_running:
                            self.mecanum_pub.publish(twist)
                        else:
                            self.tracker.pid_dist.clear()
                            self.tracker.pid_yaw.clear()
                    except Exception as e:
                        self.get_logger().error(str(e))

        if self.debug:
            if self.image_queue.full():
                # 如果队列已满，丢弃最旧的图像
                self.image_queue.get()
                # 将图像放入队列
            self.image_queue.put(result_image)
        else:
            ros_image.data = result_image.tobytes()
            self.result_publisher.publish(ros_image)

def main():
    node = OjbectTrackingNode('object_tracking')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

