#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 人体跟踪
import os
import sys
import cv2
import math
import queue
import rospy
import signal
import numpy as np
import faulthandler
import jetauto_sdk.pid as pid
import jetauto_sdk.fps as fps
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
import jetauto_sdk.misc as misc
from geometry_msgs.msg import Twist
from jetauto_interfaces.msg import ObjectsInfo
sys.path.append('/home/jetauto/jetauto_software/arm_pc')
from action_group_controller import ActionGroupController

faulthandler.enable()

class BodyControlNode:
    def __init__(self, name):
        rospy.init_node(name)
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
        camera = rospy.get_param('/depth_camera/camera_name', 'depth_cam')
        self.image_sub = rospy.Subscriber('/yolov5/object_image', Image, self.image_callback, queue_size=1)
        self.depth_image_sub = rospy.Subscriber('/%s/depth/image_raw' % camera, Image, self.depth_image_callback, queue_size=1)
        self.mecanum_pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=1)
        self.controller = ActionGroupController(use_ros=True)
        rospy.Subscriber('/yolov5/object_detect', ObjectsInfo, self.get_object_callback)
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/yolov5/init_finish'):
                    break
            except:
                rospy.sleep(0.1)
        rospy.ServiceProxy('/yolov5/start', Trigger)()
        rospy.sleep(1)
        self.controller.runAction('camera_up')
        self.mecanum_pub.publish(Twist())
        rospy.set_param('~init_finish', True)
        self.run()

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    # 获取目标检测结果
    def get_object_callback(self, msg):
        for i in msg.objects:
            class_name = i.class_name
            if class_name == 'person':
                if i.box[1] < 10:
                    self.center = [int((i.box[0] + i.box[2])/2), int(i.box[1]) + abs(int((i.box[1] - i.box[3])/4))]
                else:
                    self.center = [int((i.box[0] + i.box[2])/2), int(i.box[1]) + abs(int((i.box[1] - i.box[3])/3))]

    def depth_image_callback(self, depth_image):
        self.depth_frame = np.ndarray(shape=(depth_image.height, depth_image.width), dtype=np.uint16, buffer=depth_image.data)

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the custom image message into image)  into)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(rgb_image)

    def image_proc(self, bgr_image):
        # bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        twist = Twist()
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
            roi = self.depth_frame[h_1:h_2, w_1:w_2]
            distances = roi[np.logical_and(roi > 0, roi < 40000)]
            if distances != []:
                distance = int(np.mean(distances)/10)
            else:
                distance = 0
                #print(distance)
            ################
            if distance > 600: 
                distance = 600
            elif distance < 60:
                distance = 60
            
            #self.pid_d.SetPoint = 150
            if abs(distance - 150) < 15:
                distance = 150
            self.pid_d.update(distance - 150)  # 更新pid(update pid)
            #tmp = self.go_speed - self.pid_d.output
            self.linear_x = -misc.set_range(self.pid_d.output, -0.4, 0.4)
            #if tmp > 0.6:
            #    self.linear_x = 0.6
            #if tmp < -0.6:
            #    self.linear_x = -0.6
            #if abs(tmp) < 0.008:
            #    self.linear_x = 0
            #twist.linear.x = self.linear_x
            
            if abs(self.center[0] - w/2) < 30:
                self.center[0] = w/2
            self.pid_angular.update(self.center[0] - w/2)  # 更新pid(update pid)
            twist.linear.x = self.linear_x
            twist.angular.z = misc.set_range(self.pid_angular.output, -0.8, 0.8)
            self.mecanum_pub.publish(twist)
            #print(self.pid_angular.output)
            #print(distance, self.center)
        self.mecanum_pub.publish(twist)
        return bgr_image

    def run(self):
        while self.running:
            image = self.image_queue.get(block=True, timeout=1)
            try:
                result_image = self.image_proc(np.copy(image))
            except BaseException as e:
                print(e)
                result_image = cv2.flip(image, 1)
            self.center = None
            cv2.imshow(self.name, result_image)
            key = cv2.waitKey(1)
            if key != -1:
                self.mecanum_pub.publish(Twist())
                rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    print('\n******Press any key to exit!******')
    BodyControlNode('body_control')
