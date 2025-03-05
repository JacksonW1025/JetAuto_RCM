#!/usr/bin/env python3
# encoding: utf-8
# 指尖轨迹点发布
import cv2
import time
import enum
import rclpy
import queue
import threading
import numpy as np
import faulthandler
import sdk.fps as fps
import mediapipe as mp
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from interfaces.msg import Points, PixelPosition
from sdk.common import vector_2d_angle, distance

faulthandler.enable()

def get_hand_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标
    :param img: 像素坐标对应的图片
    :param landmarks: 归一化的关键点
    :return:
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)

def hand_angle(landmarks):
    """
    计算各个手指的弯曲角度
    :param landmarks: 手部关键点
    :return: 各个手指的角度
    """
    angle_list = []
    # thumb 大拇指
    angle_ = vector_2d_angle(landmarks[3] - landmarks[4], landmarks[0] - landmarks[2])
    angle_list.append(angle_)
    # index 食指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[6], landmarks[7] - landmarks[8])
    angle_list.append(angle_)
    # middle 中指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[10], landmarks[11] - landmarks[12])
    angle_list.append(angle_)
    # ring 无名指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[14], landmarks[15] - landmarks[16])
    angle_list.append(angle_)
    # pink 小拇指
    angle_ = vector_2d_angle(landmarks[0] - landmarks[18], landmarks[19] - landmarks[20])
    angle_list.append(angle_)
    angle_list = [abs(a) for a in angle_list]
    return angle_list

def h_gesture(angle_list):
    """
    通过二维特征确定手指所摆出的手势
    :param angle_list: 各个手指弯曲的角度
    :return : 手势名称字符串
    """
    thr_angle = 65.
    thr_angle_thumb = 53.
    thr_angle_s = 49.
    gesture_str = "none"
    if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "fist"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = "nico-nico-ni"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "hand_heart"
    elif (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "one"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
        gesture_str = "two"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] > thr_angle):
        gesture_str = "three"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "OK"
    elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "four"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
            angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
        gesture_str = "five"
    elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
            angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
        gesture_str = "six"
    else:
        "none"
    return gesture_str

class State(enum.Enum):
    NULL = 0
    START = 1
    TRACKING = 2
    RUNNING = 3

def draw_points(img, points, thickness=4, color=(0, 0, 255)):
    points = np.array(points).astype(dtype=int)
    if len(points) > 2:
        for i, p in enumerate(points):
            if i + 1 >= len(points):
                break
            cv2.line(img, p, points[i + 1], color, thickness)

class HandTrajectoryNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.drawing = mp.solutions.drawing_utils

        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_tracking_confidence=0.05,
            min_detection_confidence=0.6
        )

        self.name = name
        self.start = False
        self.running = True
        self.image = None
        self.fps = fps.FPS()  # fps计算器
        self.state = State.NULL
        self.points = []
        self.count = 0

        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.camera = 'depth_cam'
        self.image_sub = self.create_subscription(Image, '/%s/rgb/image_raw' % self.camera, self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
        self.point_publisher = self.create_publisher(Points, '~/points', 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)  # 图像处理结果发布(publish the image processing result)

        self.create_service(Trigger, '~/start', self.start_srv_callback)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback)

        self.debug = self.get_parameter('debug').value
        if self.get_parameter('start').value:
            self.start_srv_callback(Trigger.Request(), Trigger.Response())

        threading.Thread(target=self.image_proc, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start hand trajectory')
        self.start = True

        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'stop hand trajectory')
        self.start = False

        response.success = True
        response.message = "stop"
        return response

    def image_proc(self):
        points_list = []
        while self.running:
            t1 = time.time()
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            image_flip = cv2.flip(image, 1)
            bgr_image = cv2.cvtColor(image_flip, cv2.COLOR_RGB2BGR)
            if self.start:
                try:
                    cv2.waitKey(10)
                    results = self.hand_detector.process(image_flip)
                    if results is not None and results.multi_hand_landmarks:
                        gesture = "none"
                        index_finger_tip = [0, 0]
                        for hand_landmarks in results.multi_hand_landmarks:
                            self.drawing.draw_landmarks(
                                bgr_image,
                                hand_landmarks,
                                mp.solutions.hands.HAND_CONNECTIONS)
                            landmarks = get_hand_landmarks(image_flip, hand_landmarks.landmark)
                            angle_list = (hand_angle(landmarks))
                            gesture = (h_gesture(angle_list))
                            index_finger_tip = landmarks[8].tolist()
                        if self.state != State.TRACKING:
                            if gesture == "one":  # 检测食指手势， 开始指尖追踪
                                self.count += 1
                                if self.count > 5:
                                    self.count = 0
                                    self.state = State.TRACKING
                                    self.points = []
                                    points_list = []
                            else:
                                self.count = 0

                        elif self.state == State.TRACKING:
                            if gesture != "two":
                                if len(self.points) > 0:
                                    last_point = self.points[-1]
                                    if distance(last_point, index_finger_tip) < 5:
                                        self.count += 1
                                    else:
                                        self.count = 0
                                        pixels = PixelPosition()
                                        pixels.x = float(round(index_finger_tip[0]))
                                        pixels.y = float(round(index_finger_tip[1]))
                                        points_list.append(pixels)
                                        self.points.append(index_finger_tip)
                                else:
                                    pixels = PixelPosition()
                                    pixels.x = float(round(index_finger_tip[0]))
                                    pixels.y = float(round(index_finger_tip[1]))
                                    points_list.append(pixels)
                                    self.points.append(index_finger_tip)
                            draw_points(bgr_image, self.points)
                        if gesture == "five":
                            self.state = State.NULL
                            if points_list:
                                points = Points()
                                points.points = points_list
                                self.point_publisher.publish(points)
                            self.points = []
                            points_list = []
                            draw_points(bgr_image, self.points)
                except Exception as e:
                    self.get_logger().info('\033[1;32m%s\033[0m' % e)

            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(bgr_image, "bgr8"))
            if self.debug:
                cv2.imshow(self.name, bgr_image)
                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:  # 按q或者esc退出
                    break

            t2 = time.time()
            t = t2 - t1
            if t < 0.03:
                time.sleep(0.03 - t)

        rclpy.shutdown()
    
    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(rgb_image)

def main():
    node = HandTrajectoryNode('hand_trajectory')
    rclpy.spin(node)
    node.destroy_node()

if __name__ == "__main__":
    main()
