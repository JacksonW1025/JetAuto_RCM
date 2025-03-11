#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/28
# @author:aiden
# 无人驾驶
import os
import cv2
import time
import queue
import rospy
import signal
import threading
import numpy as np
import lane_detect
import jetauto_sdk.pid as pid
import jetauto_sdk.misc as misc
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import jetauto_sdk.common as common
from jetauto_app.common import Heart
from jetauto_interfaces.msg import ObjectsInfo
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_controllers.bus_servo_control import set_servos
from jetauto_sdk.common import cv2_image2ros, colors, plot_one_box
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

class SelfDrivingNode:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.image = None
        self.is_running = True
        self.pid = pid.PID(0.01, 0.0, 0.0)
        self.param_init()
        self.image_queue = queue.Queue(maxsize=2)
        self.classes = ['go', 'right', 'park', 'red', 'green', 'crosswalk']

        self.lock = threading.RLock()
        self.colors = common.Colors()
        signal.signal(signal.SIGINT, self.shutdown)
        self.machine_type = os.environ.get('MACHINE_TYPE')
        self.lane_detect = None 
        self.mecanum_pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=1)  # 底盘控制
        self.joints_pub = rospy.Publisher('servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)  # 舵机控制
        self.result_publisher = rospy.Publisher('~image_result', Image, queue_size=1)

        self.enter_srv = rospy.Service('~enter', Trigger, self.enter_srv_callback)
        self.exit_srv = rospy.Service('~exit', Trigger, self.exit_srv_callback)
        self.set_running_srv = rospy.Service('~set_running', SetBool, self.set_running_srv_callback)
        self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(None))

        if not rospy.get_param('~only_line_follow', False):
            while not rospy.is_shutdown():
                try:
                    if rospy.get_param('/yolov5/init_finish'):
                        break
                except:
                    rospy.sleep(0.1)
            rospy.ServiceProxy('/yolov5/start', Trigger)()
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/hiwonder_servo_manager/init_finish') and rospy.get_param('/joint_states_publisher/init_finish'):
                    break
            except:
                rospy.sleep(0.1)
        set_servos(self.joints_pub, 1, ((10, 350), (5, 500), (4, 320), (3, 0), (2, 750), (1, 500)))  # 初始姿态
        rospy.sleep(1)
        self.mecanum_pub.publish(Twist())
        self.dispaly = False
        if rospy.get_param('~start', True):
            self.dispaly = True
            self.enter_srv_callback(None)
            self.set_running_srv_callback(SetBoolRequest(data=True))
        #self.park_action() 
        self.image_proc()

    def param_init(self):
        self.start = False
        self.enter = False

        self.have_turn_right = False
        self.detect_turn_right = False
        self.detect_far_lane = False
        self.park_x = -1  # 停车标识的x像素坐标

        self.start_turn_time_stamp = 0
        self.count_turn = 0
        self.start_turn = False  # 开始转弯

        self.count_right = 0
        self.count_right_miss = 0
        self.turn_right = False  # 右转标志

        self.last_park_detect = False
        self.count_park = 0
        self.stop = False  # 停下标识
        self.start_park = False  # 开始泊车标识

        self.count_crosswalk = 0
        self.crosswalk_distance = 0  # 离斑马线距离
        self.crosswalk_length = 0.1 + 0.3  # 斑马线长度 + 车长

        self.start_slow_down = False  # 减速标识
        self.normal_speed = 0.15  # 正常前进速度
        self.slow_down_speed = 0.1  # 减速行驶的速度

        self.traffic_signs_status = None  # 记录红绿灯状态
        self.red_loss_count = 0

        self.object_sub = None
        self.image_sub = None
        self.objects_info = []

    def enter_srv_callback(self, _):
        rospy.loginfo("self driving enter")
        with self.lock:
            self.start = False
            if rospy.get_param('~use_depth_cam', False):
                self.lane_detect = lane_detect.LaneDetector("yellow", 'Stereo')
                camera = rospy.get_param('/depth_camera/camera_name', 'depth_cam')  # 获取参数
                self.image_sub = rospy.Subscriber('/%s/rgb/image_raw' % camera, Image, self.image_callback)  # 摄像头订阅
            else:
                self.lane_detect = lane_detect.LaneDetector("yellow", 'Mono')
                camera = rospy.get_param('/usb_cam_name/camera_name', 'usb_cam')  # 获取参数
                self.image_sub = rospy.Subscriber('/%s/image_raw' % camera, Image, self.image_callback)  # 摄像头订阅
            self.object_sub = rospy.Subscriber('/yolov5/object_detect', ObjectsInfo, self.get_object_callback)
            self.mecanum_pub.publish(Twist())
            self.enter = True
        return TriggerResponse(success=True)

    def exit_srv_callback(self, _):
        rospy.loginfo("self driving exit")
        with self.lock:
            try:
                if self.image_sub is not None:
                    self.image_sub.unregister()
                if self.object_sub is not None:
                    self.object_sub.unregister()
            except Exception as e:
                rospy.logerr(str(e))
            self.mecanum_pub.publish(Twist())
        self.param_init()
        return TriggerResponse(success=True)

    def set_running_srv_callback(self, req: SetBoolRequest):
        rospy.loginfo("set_running")
        with self.lock:
            self.start = req.data
            if not self.start:
                self.mecanum_pub.publish(Twist())
        return SetBoolResponse(success=req.data)

    def shutdown(self, signum, frame):  # ctrl+c关闭处理
        self.is_running = False
        rospy.loginfo('shutdown')

    def image_callback(self, ros_image):  # 目标检查回调
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(rgb_image)
    
    # 泊车处理
    def park_action(self):
        twist = Twist()
        twist.linear.y = -0.2
        self.mecanum_pub.publish(twist)
        rospy.sleep(0.38/0.2)
        self.mecanum_pub.publish(Twist())

    def image_proc(self):
        while self.is_running:
            time_start = time.time()
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.is_running:
                    break
                else:
                    continue
            result_image = image.copy()
            if self.start:
                h, w = image.shape[:2]

                # 获取车道线的二值化图
                binary_image = self.lane_detect.get_binary(image)
                # cv2.imshow('1', binary_image)
                # 检测到斑马线,开启减速标志
                if 450 < self.crosswalk_distance and not self.start_slow_down:  # 只有足够近时才开始减速
                    self.count_crosswalk += 1
                    if self.count_crosswalk == 3:  # 多次判断，防止误检测
                        self.count_crosswalk = 0
                        self.start_slow_down = True  # 减速标识
                        self.count_slow_down = rospy.get_time()  # 减速固定时间
                else:  # 需要连续检测，否则重置
                    self.count_crosswalk = 0

                twist = Twist()
                # 减速行驶处理
                if self.start_slow_down:
                    if self.traffic_signs_status is not None:
                        # 通过面积判断离灯灯远近，如果太近那么即使是红灯也不会停
                        area = abs(self.traffic_signs_status.box[0] - self.traffic_signs_status.box[2]) * abs(
                            self.traffic_signs_status.box[1] - self.traffic_signs_status.box[3])
                        if self.traffic_signs_status.class_name == 'red' and area < 1000:  # 如果遇到红灯就停车
                            self.mecanum_pub.publish(Twist())
                            self.stop = True
                        elif self.traffic_signs_status.class_name == 'green':  # 遇到绿灯，速度放缓
                            twist.linear.x = self.slow_down_speed
                            self.stop = False
                    if not self.stop:  # 其他非停止的情况速度放缓， 同时计时，时间=斑马线的长度/行驶速度
                        twist.linear.x = self.slow_down_speed
                        if rospy.get_time() - self.count_slow_down > self.crosswalk_length/twist.linear.x:
                            self.start_slow_down = False
                else:
                    twist.linear.x = self.normal_speed  # 直走正常速度
                # 检测到 停车标识+斑马线 就减速, 让识别稳定
                if 0 < self.park_x and 180 < self.crosswalk_distance:
                    twist.linear.x = self.slow_down_speed
                    if not self.start_park and 340 < self.crosswalk_distance:  # 离斑马线足够近时就开启停车
                        self.mecanum_pub.publish(Twist())
                        self.start_park = True
                        self.stop = True
                        threading.Thread(target=self.park_action).start()
                
                # print(self.crosswalk_distance, self.park_x, self.start_turn) 
                # 右转及停车补线策略
                if self.detect_turn_right:
                    if 400 < self.crosswalk_distance:
                        self.detect_turn_right = False
                        self.turn_right = True
                if self.turn_right:
                    y = self.lane_detect.add_horizontal_line(binary_image)
                    if 0 < y < 400:
                        roi = [(0, y), (w, y), (w, 0), (0, 0)]
                        cv2.fillPoly(binary_image, [np.array(roi)], [0, 0, 0])  # 将上面填充为黑色，防干扰
                        min_x = cv2.minMaxLoc(binary_image)[-1][0]
                        cv2.line(binary_image, (min_x, y), (w, y), (255, 255, 255), 40)  # 画虚拟线来驱使转弯
                        # cv2.imshow('1', binary_image)
                elif 0 < self.park_x and not self.start_turn:  # 检测到停车标识需要填补线，使其保持直走
                    if not self.detect_far_lane:
                        up, down = self.lane_detect.add_vertical_line_near(binary_image)
                        binary_image[:, :] = 0  # 全置黑，防止干扰
                        if abs(up[1] - down[1]) < 30:  # 当将要看不到车道线时切换到识别较远车道线
                            self.detect_far_lane = True
                    else:
                        up, down = self.lane_detect.add_vertical_line_far(binary_image)
                        binary_image[:, :] = 0
                    if up != down:
                        cv2.line(binary_image, up, down, (255, 255, 255), 20)  # 手动画车道线

                result_image, lane_angle, lane_x = self.lane_detect(binary_image, image.copy())  # 在处理后的图上提取车道线中心
                # 巡线处理
                # cv2.imshow('2', result_image)
                if not self.stop and lane_x > 0:
                    if lane_x > 155:  # 转弯
                        if self.turn_right:
                            self.count_right_miss += 1
                            # if self.count_right_miss < 20:
                                # pass
                                # twist.angular.z = 0  # 转弯速度
                            if 1:
                                self.count_turn += 1
                                # print(6666, self.count_turn)
                                if self.count_turn > 3 and not self.start_turn:  # 稳定转弯
                                    # print(555555)
                                    self.start_turn = True
                                    self.count_turn = 0
                                    self.start_turn_time_stamp = rospy.get_time()
                                twist.angular.z = -0.45  # 转弯速度
                            # print(7777, self.count_right_miss)
                            if self.count_right_miss >= 60:
                                self.count_right_miss = 0
                                self.turn_right = False
                        else:
                            self.count_turn += 1
                            if self.count_turn > 5 and not self.start_turn:  # 稳定转弯
                                self.start_turn = True
                                self.count_turn = 0
                                self.start_turn_time_stamp = rospy.get_time()
                            twist.angular.z = -0.45  # 转弯速度
                    else:  # 直道由pid计算转弯修正
                        self.count_turn = 0
                        if rospy.get_time() - self.start_turn_time_stamp > 3 and self.start_turn:
                            self.start_turn = False
                        if not self.start_turn:
                            self.pid.SetPoint = 70  # 在车道中间时线的坐标
                            if abs(lane_x - 70) < 10:
                                lane_x = 70
                            self.pid.update(lane_x)
                            twist.angular.z = misc.set_range(self.pid.output, -0.2, 0.2)
                    self.mecanum_pub.publish(twist)
                else:
                    self.pid.clear()

                if self.objects_info != []:
                    for i in self.objects_info:
                        box = i.box
                        class_name = i.class_name
                        cls_conf = i.score
                        cls_id = self.classes.index(class_name)
                        color = colors(cls_id, True)
                        plot_one_box(
                            box,
                            result_image,
                            color=color,
                            label="{}:{:.2f}".format(class_name, cls_conf),
                        )
            result_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            if self.dispaly:
                cv2.imshow('result', result_image)
                key = cv2.waitKey(1)
                if key != -1:
                    self.is_running = False

            #ros_image.data = result_image.tostring()
            self.result_publisher.publish(cv2_image2ros(result_image))
            time_d = 0.03 - (time.time() - time_start)
            if time_d > 0:
                time.sleep(time_d)
        self.mecanum_pub.publish(Twist())

    # 获取目标检测结果
    def get_object_callback(self, msg):
        self.objects_info = msg.objects
        if self.objects_info == []:  # 没有识别到时重置变量
            self.traffic_signs_status = None
            self.crosswalk_distance = 0
        else:
            min_distance = 0
            for i in self.objects_info:
                class_name = i.class_name
                center = (int((i.box[0] + i.box[2])/2), int((i.box[1] + i.box[3])/2))
                
                if class_name == 'crosswalk':  
                    if center[1] > min_distance:  # 获取最近的人行道y轴像素坐标
                        min_distance = center[1]
                elif class_name == 'right':  # 获取右转标识
                    self.count_right += 1
                    self.count_right_miss = 0
                    if self.count_right >= 5:  # 检测到多次就将右转标志至真
                        self.detect_turn_right = True
                        self.count_right = 0
                elif class_name == 'park':  # 获取停车标识中心坐标
                    self.park_x = center[0]
                elif class_name == 'red' or class_name == 'green':  # 获取红绿灯状态
                    self.traffic_signs_status = i
        
            self.crosswalk_distance = min_distance

if __name__ == "__main__":
    SelfDrivingNode('self_driving')
