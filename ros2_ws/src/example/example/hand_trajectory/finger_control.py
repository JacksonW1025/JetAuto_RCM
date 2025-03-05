#!/usr/bin/env python3
# encoding: utf-8
# 指尖轨迹点发布
import cv2
import time
import math
import queue
import rclpy
import threading
import numpy as np
import sdk.fps as fps
import mediapipe as mp
from rclpy.node import Node
from cv_bridge import CvBridge
from kinematics import transform
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from kinematics_msgs.srv import SetRobotPose
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from kinematics.kinematics_control import set_pose_target
from servo_controller.bus_servo_control import set_servo_position

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

class FingerControlNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.drawing = mp.solutions.drawing_utils
        self.image_queue = queue.Queue(maxsize=2)
        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_tracking_confidence=0.05,
            min_detection_confidence=0.6
        )

        self.name = name
        self.running = True
        self.fps = fps.FPS()  # fps计算器
        self.z_dis = 0.36
        self.y_dis = 500
        self.last_d = 0
        self.x_init = transform.link3 + transform.tool_link
        self.bridge = CvBridge()
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) # 舵机控制
        self.camera = 'depth_cam'
        self.image_sub = self.create_subscription(Image, '/%s/rgb/image_raw' % self.camera, self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
        self.client = self.create_client(Trigger, '/kinematics/init_finish')
        self.client.wait_for_service()
        timer_cb_group = ReentrantCallbackGroup()
        self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target', callback_group=timer_cb_group)
        self.kinematics_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()

        self.init_action()
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def init_action(self):
        msg = set_pose_target([self.x_init, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
        res = self.send_request(self.kinematics_client, msg)
        if res.pulse:
            servo_data = res.pulse
            set_servo_position(self.joints_pub, 1.5, ((10, 500), (5, 500), (4, servo_data[3]), (3, servo_data[2]), (2, servo_data[1]), (1, servo_data[0])))
            time.sleep(1.8)
        self.mecanum_pub.publish(Twist())

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def main(self):
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
            results = self.hand_detector.process(image_flip)
            if results is not None and results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    self.drawing.draw_landmarks(
                        bgr_image,
                        hand_landmarks,
                        mp.solutions.hands.HAND_CONNECTIONS)
                    landmarks = get_hand_landmarks(image_flip, hand_landmarks.landmark)
                    try:
                        index_finger_tip = landmarks[8].tolist()
                        thumb_finger_tip = landmarks[4].tolist() 
                        cv2.circle(bgr_image, (int(index_finger_tip[0]), int(index_finger_tip[1])), 10, (0, 255, 255), -1)
                        cv2.circle(bgr_image, (int(thumb_finger_tip[0]), int(thumb_finger_tip[1])), 10, (0, 255, 255), -1)
                        cv2.line(bgr_image, (int(index_finger_tip[0]), int(index_finger_tip[1])), (int(thumb_finger_tip[0]), int(thumb_finger_tip[1])), (0, 255, 255), 5)
                        
                        d = math.sqrt(math.pow(thumb_finger_tip[0] - index_finger_tip[0], 2) + math.pow(thumb_finger_tip[1] - index_finger_tip[1], 2)) 
                        
                        # print(d, d - self.last_d)
                        if abs(d - self.last_d) > 10:
                            if d -self.last_d > 0:
                                self.z_dis += 0.01
                            else:
                                self.z_dis -= 0.01
                            
                            if self.z_dis > 0.46:
                                self.z_dis = 0.46
                            if self.z_dis < 0.36:
                                self.z_dis = 0.36
                            msg = set_pose_target([self.x_init, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
                            res = self.send_request(self.kinematics_client, msg)
                            t2 = time.time()
                            t = t2 - t1
                            if t < 0.02:
                                time.sleep(0.02 - t)
                            if res.pulse:
                                servo_data = res.pulse
                                set_servo_position(self.joints_pub, 0.02, ((10, 500), (5, 500), (4, servo_data[3]), (3, servo_data[2]), (2, servo_data[1]), (1, int(self.y_dis))))
                        self.last_d = d
                    except BaseException as e:
                        self.get_logger().info('\033[1;32m%s\033[0m' % e)

            cv2.imshow(self.name, bgr_image)
            key = cv2.waitKey(10)
            if key == ord('q') or key == 27:  # 按q或者esc退出
                break

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
    node = FingerControlNode('finger_control')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
