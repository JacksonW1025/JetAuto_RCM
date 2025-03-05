#!/usr/bin/env python3
# encoding: utf-8
# 雷达避障，跟随功能(lidar obstacle avoidance and lidar following function)
import os
import math
import time
import rclpy
import threading
import numpy as np
import sdk.pid as pid
import sdk.common as common
from rclpy.node import Node
# from app.common import Heart
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from interfaces.srv import SetInt64, SetFloat64List
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position

CAR_WIDTH = 0.4  # meter
MAX_SCAN_ANGLE = 240  # 激光的扫描角度,去掉总是被遮挡的部分degree(the scanning angle of Lidar. The covered part is always eliminated)

class LidarController(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.name = name
        self.running_mode = 0
        self.threshold = 0.6  # meters
        self.scan_angle = math.radians(90)  # radians
        self.speed = 0.2
        self.last_act = 0
        self.timestamp = 0
        self.angle_data = []
        # pid参数
        self.pid_yaw = pid.PID(1.6, 0, 0.16)
        self.pid_dist = pid.PID(1.7, 0, 0.16)
        self.lock = threading.RLock()
        self.lidar_sub = None
        self.lidar_type = os.environ.get('LIDAR_TYPE')
        self.machine_type = os.environ.get('MACHINE_TYPE')
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # 底盘控制(chassis control)
        self.create_service(Trigger, '~/enter', self.enter_srv_callback)  # 进入玩法(enter the game)
        self.create_service(Trigger, '~/exit', self.exit_srv_callback)  # 退出玩法(exit the game)
        self.create_service(SetInt64, '~/set_running', self.set_running_srv_callback)  # 开启玩法(start the game)
        #Heart(self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(None))  # 心跳包(heartbeat package)
        self.create_service(SetFloat64List, '~/set_parameter', self.set_parameters_srv_callback)  # 参数设置(set parameter)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def reset_value(self):
        '''
        重置参数(reset parameter)
        :return:
        '''
        self.running_mode = 0
        self.threshold = 0.6
        self.scan_angle = math.radians(90)
        self.speed = 0.2
        self.last_act = 0
        self.speed = 0.2
        self.timestamp = 0
        self.pid_yaw.clear()
        self.pid_dist.clear()
        try:
            if self.lidar_sub is not None:
                self.lidar_sub.unregister()
        except Exception as e:
            self.get_logger().error(str(e))

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'lidar enter')
        self.reset_value()
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)  # 订阅雷达数据(subscribe to Lidar data)
        set_servo_position(self.joints_pub, 1, ((10, 300), (5, 500), (4, 210), (3, 40), (2, 665), (1, 500)))
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'lidar exit')
        self.reset_value()
        self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "exit"
        return response

    def set_running_srv_callback(self, request, response):
        '''
        开启不同功能(enable different functions)
        :param req: 0关闭，1避障，2跟随，3警卫(0 close, 1 obstacle avoidance, 2 Lidar following, 3 Lidar guarding)
        :return:
        '''
        new_running_mode = request.data
        self.get_logger().info('\033[1;32m%s\033[0m' % ("set_running " + str(new_running_mode)))
        if not 0 <= new_running_mode <= 3:
            response.success = False
            response.message = "Invalid running mode {}".format(new_running_mode)
        else:
            with self.lock:
                self.running_mode = new_running_mode
        self.mecanum_pub.publish(Twist())
        return response

    def set_parameters_srv_callback(self, request, response):
        '''
        设置避障阈值，速度参数(set the threshold of obstacle avoidance and speed)
        :param req:
        :return:
        '''
        new_parameters = request.data
        new_threshold, new_scan_angle, new_speed = new_parameters
        self.get_logger().info("\033[1;32mn_t:{:2f}, n_a:{:2f}, n_s:{:2f}\033[0m".format(new_threshold, new_scan_angle, new_speed))
        if not 0.3 <= new_threshold <= 1.5:
            response.success = False
            response.message = "New threshold ({:.2f}) is out of range (0.3 ~ 1.5)".format(new_threshold)
            return response
        if not 0 <= new_scan_angle <= 90:
            response.success = False
            response.message = "New scan angle ({:.2f}) is out of range (0 ~ 90)"
            return response
        if not new_speed > 0:
            response.success = False
            response.message = "Invalid speed"
            return response

        with self.lock:
            self.threshold = new_threshold
            self.scan_angle = math.radians(new_scan_angle)
            self.speed = new_speed
            self.speed = self.speed
        return response

    def lidar_callback(self, lidar_data):
        # 雷达订阅回调(Lidar subscription callback)
        twist = Twist()
        # 数据大小 = 扫描角度/每扫描一次增加的角度(data size= scanning angle/ the increased angle per scan)
        if self.lidar_type != 'G4':
            max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[:max_index]  # 左半边数据 (the left data)
            right_ranges = lidar_data.ranges[::-1][:max_index]  # 右半边数据 (the right data)
        elif self.lidar_type == 'G4':
            min_index = int(math.radians((360 - MAX_SCAN_ANGLE) / 2.0) / lidar_data.angle_increment)
            max_index = int(math.radians(180) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[::-1][min_index:max_index][::-1]  # 左半边数据 (the left data)
            right_ranges = lidar_data.ranges[min_index:max_index][::-1]  # 右半边数据 (the right data)
        with self.lock:
            # 根据设定取数据 (get the data according to the settings)
            angle = self.scan_angle / 2
            angle_index = int(angle / lidar_data.angle_increment + 0.50)
            left_range, right_range = np.array(left_ranges[:angle_index]), np.array(right_ranges[:angle_index])

            if self.running_mode == 1 and self.timestamp <= time.time():
                left_nonzero = left_range.nonzero()
                right_nonzero = right_range.nonzero()
                left_nonan = ~np.isnan(left_range[left_nonzero])
                right_nonan = ~np.isnan(right_range[right_nonzero])
                min_dist_left_ = left_range[left_nonzero][left_nonan]
                min_dist_right_ = right_range[right_nonzero][right_nonan]
                # 取左右最近的距离(Take the nearest distance left and right)
                if len(min_dist_left_) > 1 and len(min_dist_right_) > 1:
                    min_dist_left = min_dist_left_.min()
                    min_dist_right = min_dist_right_.min()
                    if min_dist_left <= self.threshold and min_dist_right > self.threshold:  # 左侧有障碍(there is obstacle at left)
                        twist.linear.x = self.speed / 6  # 转弯时向前速度尽量小(keep the forward velocity as low as possible when turning)
                        max_angle = math.radians(90)  # 转90度(turn 90 degree)
                        w = self.speed * 6.0  # 转弯速度尽量大(keep the turning speed as high as possible)
                        twist.angular.z = -w
                        if self.last_act != 0 and self.last_act != 1:
                            twist.angular.z = w
                        self.last_act = 1
                        self.mecanum_pub.publish(twist)
                        self.timestamp = time.time() + (max_angle / w / 2)
                    elif min_dist_left <= self.threshold and min_dist_right <= self.threshold:  # 两侧都有障碍
                        twist.linear.x = self.speed / 6  # 转弯时向前速度尽量小(keep the forward velocity as low as possible when turning)
                        w = self.speed * 6.0  # 转弯速度尽量大(keep the turning speed as high as possible)
                        twist.angular.z = w
                        self.last_act = 3
                        self.mecanum_pub.publish(twist)
                        # 转180度
                        self.timestamp = time.time() + (math.radians(180) / w / 2)
                    elif min_dist_left > self.threshold and min_dist_right <= self.threshold:  # 右侧有障碍(there is obstacles at right)
                        twist.linear.x = self.speed / 6  # 转弯时向前速度尽量小(keep the forward velocity as low as possible when turning)
                        max_angle = math.radians(90)  # 转90度 turn 90 degrees
                        w = self.speed * 6.0  # 转弯速度尽量大(keep the turning speed as high as possible)
                        twist.angular.z = w
                        if self.last_act != 0 and self.last_act != 2:
                            twist.angular.z = -w
                        self.last_act = 2
                        self.mecanum_pub.publish(twist)
                        self.timestamp = time.time() + (max_angle / w / 2)
                    else:  # 没有障碍(none obstacle)
                        self.last_act = 0
                        twist.linear.x = self.speed  # 直行
                        self.mecanum_pub.publish(twist)
            elif self.running_mode == 2:
                # 拼合距离数据, 从右半侧逆时针到左半侧(the merged distance data from right half counterclockwise to the left half)
                ranges = np.append(right_range[::-1], left_range)
                nonzero = ranges.nonzero()
                nonan = ~np.isnan(ranges[nonzero])
                dist_ = ranges[nonzero][nonan]
                if len(dist_) > 1:
                    dist = dist_.min()
                    min_index = list(ranges).index(dist)
                    angle = -angle + lidar_data.angle_increment * min_index  # 计算最小值对应的角度(calculate the angle corresponding to the minimum value)
                    if dist < self.threshold and abs(math.degrees(angle)) > 5:  # 控制左右(control the left and right)
                        if self.lidar_type != 'G4':
                            self.pid_yaw.update(-angle)
                            twist.angular.z = common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0)
                        elif self.lidar_type == 'G4':
                            self.pid_yaw.update(angle)
                            twist.angular.z = -common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0)
                    else:
                        self.pid_yaw.clear()

                    if dist < self.threshold and abs(0.2 - dist) > 0.02:  # 控制前后(control the front and back)
                        self.pid_dist.update(self.threshold / 2 - dist)
                        twist.linear.x = common.set_range(self.pid_dist.output, -self.speed, self.speed)
                    else:
                        self.pid_dist.clear()
                    if abs(twist.angular.z) < 0.008:
                        twist.angular.z = 0.0
                    if abs(twist.linear.x) < 0.05:
                        twist.linear.x = 0.0
                    self.mecanum_pub.publish(twist)
            elif self.running_mode == 3:
                # 拼合距离数据, 从右半侧逆时针到左半侧(the merged distance data from right half counterclockwise to the left half)
                ranges = np.append(right_range[::-1], left_range)
                nonzero = ranges.nonzero()
                nonan = ~np.isnan(ranges[nonzero])
                dist_ = ranges[nonzero][nonan]
                if len(dist_) > 1:
                    dist = dist_.min()
                    min_index = list(ranges).index(dist)
                    angle = -angle + lidar_data.angle_increment * min_index  # 计算最小值对应的角度(calculate the angle corresponding to the minimum value)
                    if dist < self.threshold and abs(math.degrees(angle)) > 5:  # 控制左右(control the left and right)
                        if self.lidar_type != 'G4':
                            self.pid_yaw.update(-angle)
                            twist.angular.z = common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0)
                        elif self.lidar_type == 'G4':
                            self.pid_yaw.update(angle)
                            twist.angular.z = -common.set_range(self.pid_yaw.output, -self.speed * 6.0, self.speed * 6.0)
                    else:
                        self.pid_yaw.clear()
                    if abs(twist.angular.z) < 0.008:
                        twist.angular.z = 0.0
                    self.mecanum_pub.publish(twist)

def main():
    node = LidarController('lidar_controller')
    rclpy.spin(node)  # 循环等待ROS2退出

if __name__ == "__main__":
    main()
