#!/usr/bin/env python3
# encoding: utf-8
# 巡线(line following)
import os
import cv2
import math
import rclpy
import queue
import threading
import numpy as np
import sdk.pid as pid
import sdk.common as common
from rclpy.node import Node
from cv_bridge import CvBridge
from app.common import ColorPicker
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import Image, LaserScan
from interfaces.srv import SetPoint, SetFloat64
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position

MAX_SCAN_ANGLE = 240  # 激光的扫描角度,去掉总是被遮挡的部分degree(the scanning angle of lidar. The covered part is always eliminated)
class LineFollower:
    def __init__(self, color, node):
        self.node = node
        self.target_lab, self.target_rgb = color
        self.machine_type = os.environ.get('MACHINE_TYPE')
        if self.machine_type == 'JetAuto':
            self.rois = ((0.9375, 1.0, 0.0, 1.0, 0.7), (0.8125, 0.875, 0.0, 1.0, 0.2), (0.6875, 0.75, 0.0, 1.0, 0.1))
        else:
            self.rois = ((0.77, 0.833, 0.0, 1.0, 0.9), (0.645, 0.708, 0.0, 1.0, 0.1), (0.52, 0.583, 0.0, 1.0, 0.0))
        self.weight_sum = 1.0

    @staticmethod
    def get_area_max_contour(contours, threshold=100):
        '''
        获取最大面积对应的轮廓(get the contour of the largest area)
        :param contours:
        :param threshold:
        :return:
        '''
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
        if len(contour_area) > 0:
            max_c_a = max(contour_area, key=lambda c_a: c_a[1])
            return max_c_a
        return None

    def __call__(self, image, result_image, threshold):
        centroid_sum = 0
        h, w = image.shape[:2]
        min_color = [int(self.target_lab[0] - 50 * threshold * 2),
                     int(self.target_lab[1] - 50 * threshold),
                     int(self.target_lab[2] - 50 * threshold)]
        max_color = [int(self.target_lab[0] + 50 * threshold * 2),
                     int(self.target_lab[1] + 50 * threshold),
                     int(self.target_lab[2] + 50 * threshold)]
        target_color = self.target_lab, min_color, max_color
        for roi in self.rois:
            blob = image[int(roi[0]*h):int(roi[1]*h), int(roi[2]*w):int(roi[3]*w)]  # 截取roi(intercept roi)
            img_lab = cv2.cvtColor(blob, cv2.COLOR_RGB2LAB)  # rgb转lab(convert rgb into lab)
            img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # 高斯模糊去噪(perform Gaussian filtering to reduce noise)
            mask = cv2.inRange(img_blur, tuple(target_color[1]), tuple(target_color[2]))  # 二值化(image binarization)
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(corrode)
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)
            # cv2.imshow('section:{}:{}'.format(roi[0], roi[1]), cv2.cvtColor(dilated, cv2.COLOR_GRAY2BGR))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓(find the contour)
            max_contour_area = self.get_area_max_contour(contours, 30)  # 获取最大面积对应轮廓(get the contour corresponding to the largest contour)
            if max_contour_area is not None:
                rect = cv2.minAreaRect(max_contour_area[0])  # 最小外接矩形(minimum circumscribed rectangle)
                box = np.intp(cv2.boxPoints(rect))  # 四个角(four corners)
                for j in range(4):
                    box[j, 1] = box[j, 1] + int(roi[0]*h)
                cv2.drawContours(result_image, [box], -1, (0, 255, 255), 2)  # 画出四个点组成的矩形(draw the rectangle composed of four points)

                # 获取矩形对角点(acquire the diagonal points of the rectangle)
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                # 线的中心点(center point of the line)
                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)   # 画出中心点(draw the center point)
                centroid_sum += line_center_x * roi[-1]
        if centroid_sum == 0:
            return result_image, None
        center_pos = centroid_sum / self.weight_sum  # 按比重计算中心点(calculate the center point according to the ratio)
        deflection_angle = -math.atan((center_pos - (w / 2.0)) / (h / 2.0))   # 计算线角度(calculate the line angle)
        return result_image, deflection_angle

class LineFollowingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        self.name = name
        self.set_callback = False
        self.is_running = False
        self.color_picker = None
        self.follower = None
        self.scan_angle = math.radians(45)
        self.pid = pid.PID(0.01, 0.0, 0.0)
        self.empty = 0
        self.count = 0
        self.stop = False
        self.threshold = 0.5
        self.stop_threshold = 0.4
        self.lock = threading.RLock()
        self.image_sub = None
        self.lidar_sub = None
        self.image_height = None
        self.image_width = None
        
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(2)
        self.lidar_type = os.environ.get('LIDAR_TYPE')
        self.machine_type = os.environ.get('MACHINE_TYPE')
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # 底盘控制(chassis control)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # 图像处理结果发布(publish the image processing result)
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)  # 进入玩法(enter the game)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)  # 退出玩法(exit the game)
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)  # 开启玩法(start the game)
        self.create_service(SetPoint, '~/set_target_color', self.set_target_color_srv_callback)  # 设置颜色(set the color)
        self.create_service(Trigger, '~/get_target_color', self.get_target_color_srv_callback)   # 获取颜色(get the color)
        self.create_service(SetFloat64, '~/set_threshold', self.set_threshold_srv_callback)  # 设置阈值(set the threshold)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        # self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(None))  # 心跳包(heartbeat package)
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
        self.get_logger().info('\033[1;32m%s\033[0m' % "line following enter")
        try:
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                # self.image_sub.unregister()
            if self.lidar_sub is not None:
                self.destroy_subscription(self.lidar_sub)
                # self.lidar_sub.unregister()
        except Exception as e:
            self.get_logger().error(str(e))
        with self.lock:
            self.stop = False
            self.is_running = False
            self.color_picker = None
            self.pid = pid.PID(1.1, 0.0, 0.0)
            self.follower = None
            self.threshold = 0.5
            self.empty = 0
            if self.machine_type == 'JetAuto':
                self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
            else:
                self.image_sub = self.create_subscription(Image, '/usb_cam/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
            self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)  # 订阅雷达(subscribe to Lidar)
            set_servo_position(self.joints_pub, 1, ((10, 300), (5, 500), (4, 210), (3, 40), (2, 665), (1, 500)))
            self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "line following exit")
        try:
            if self.image_sub is not None:
                self.image_sub.unregister()
            if self.lidar_sub is not None:
                self.lidar_sub.unregister()
        except Exception as e:
            self.get_logger().error(str(e))
        with self.lock:
            self.is_running = False
            self.color_picker = None
            self.pid = pid.PID(0.01, 0.0, 0.0)
            self.follower = None
            self.threshold = 0.5
            self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "exit"
        return response

    def set_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_target_color")
        with self.lock:
            x, y = request.data.x, request.data.y
            self.follower = None
            if x == -1 and y == -1:
                self.color_picker = None
            else:
                self.color_picker = ColorPicker(request.data, 20)
                self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "set_target_color"
        return response

    def get_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "get_target_color")
        response.success = False
        response.message = "get_target_color"
        with self.lock:
            if self.follower is not None:
                response.success = True
                rgb = self.follower.target_rgb
                response.message = "{},{},{}".format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
        return response

    def set_running_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_running")
        with self.lock:
            self.is_running = request.data
            self.empty = 0
            if not self.is_running:
                self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "set_running"
        return response

    def set_threshold_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set threshold")
        with self.lock:
            self.threshold = request.data
            response.success = True
            response.message = "set_threshold"
            return response

    def lidar_callback(self, lidar_data):
        # 数据大小 = 扫描角度/每扫描一次增加的角度(data size= scanning angle/ the increased angle per scan)
        if self.lidar_type != 'G4':
            min_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
            max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[:max_index]  # 左半边数据(left data)
            right_ranges = lidar_data.ranges[::-1][:max_index]  # 右半边数据(right data)
        elif self.lidar_type == 'G4':
            min_index = int(math.radians((360 - MAX_SCAN_ANGLE) / 2.0) / lidar_data.angle_increment)
            max_index = int(math.radians(180) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[::-1][min_index:max_index][::-1]  # 左半边数据(left data)
            right_ranges = lidar_data.ranges[min_index:max_index][::-1]  # 右半边数据(right data)

        # 根据设定取数据(Get data according to settings)
        angle = self.scan_angle / 2
        angle_index = int(angle / lidar_data.angle_increment + 0.50)
        left_range, right_range = np.array(left_ranges[:angle_index]), np.array(right_ranges[:angle_index])

        left_nonzero = left_range.nonzero()
        right_nonzero = right_range.nonzero()
        left_nonan = ~np.isnan(left_range[left_nonzero])
        right_nonan = ~np.isnan(right_range[right_nonzero])
        # 取左右最近的距离(Take the nearest distance left and right)
        min_dist_left_ = left_range[left_nonzero][left_nonan]
        min_dist_right_ = right_range[right_nonzero][right_nonan]
        if len(min_dist_left_) > 1 and len(min_dist_right_) > 1:
            min_dist_left = min_dist_left_.min()
            min_dist_right = min_dist_right_.min()
            if min_dist_left < self.stop_threshold or min_dist_right < self.stop_threshold:
                self.stop = True
            else:
                self.count += 1
                if self.count > 5:
                    self.count = 0
                    self.stop = False

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        cv_image = np.array(cv_image, dtype=np.uint8)
        self.image_height, self.image_width = cv_image.shape[:2]
        result_image = np.copy(cv_image)  # 显示结果用的画面 (the image used to display the result)
        with self.lock:
            # 颜色拾取器和识别巡线互斥, 如果拾取器存在就开始拾取(color picker and line recognition are exclusive. If there is color picker, start picking)
            if self.color_picker is not None:  # 拾取器存在(color picker exists)
                try:
                    target_color, result_image = self.color_picker(cv_image, result_image)
                    if target_color is not None:
                        self.color_picker = None
                        self.get_logger().info("target color: {}".format(target_color))
                        self.follower = LineFollower(target_color, self) 
                except Exception as e:
                    self.get_logger().error(str(e))
            else:
                twist = Twist()
                twist.linear.x = 0.15
                if self.follower is not None:
                    try:
                        result_image, deflection_angle = self.follower(cv_image, result_image, self.threshold)
                        if deflection_angle is not None and self.is_running and not self.stop:
                            self.pid.update(deflection_angle)
                            twist.angular.z = common.set_range(-self.pid.output, -1.0, 1.0)
                            self.mecanum_pub.publish(twist)
                        elif self.stop:
                            self.mecanum_pub.publish(Twist())
                        else:
                            self.pid.clear()
                    except Exception as e:
                        self.get_logger().error(str(e))
        if self.debug:
            if self.image_queue.full():
                # 如果队列已满，丢弃最旧的图像
                self.image_queue.get()
                # 将图像放入队列
            self.image_queue.put(result_image)
        else:
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))

def main():
    node = LineFollowingNode('line_following')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

