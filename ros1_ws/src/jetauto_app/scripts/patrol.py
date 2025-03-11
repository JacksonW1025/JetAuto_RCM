#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/03/24
# @author:aiden
# 智能巡逻(intelligent patrolling)
import math
import rospy
import threading
import geometry_msgs.msg as geo_msg
from jetauto_app.common import Heart
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from jetauto_interfaces.srv import SetInt64, SetInt64Request, SetInt64Response

class PatrolController:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.running_mode = 0
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.rectangle_h = 0.5
        self.rectangle_w = 0.5
        self.triangle_l = 0.5
        self.th = None
        self.last_mode = 0
        self.thread_running = True
        self.mecanum_pub = rospy.Publisher('/jetauto_controller/cmd_vel', geo_msg.Twist,
                                           queue_size=1)  # 底盘控制(chassis control)
        self.enter_srv = rospy.Service(self.name + '/enter', Trigger, self.enter_srv_callback)  # 进入玩法(enter the game)
        self.exit_srv = rospy.Service(self.name + '/exit', Trigger, self.exit_srv_callback)  # 退出玩法(exit the game)
        self.set_running_srv = rospy.Service(self.name + '/set_running', SetInt64,
                                             self.set_running_srv_callback)  # 开启玩法(start the game)
        self.heart = Heart(self.name + '/heartbeat', 5,
                           lambda _: self.exit_srv_callback(None))  # 心跳包(heartbeat package)
        rospy.sleep(0.2)
        self.mecanum_pub.publish(geo_msg.Twist())

    def reset_value(self):
        '''
        重置参数(reset parameter)
        :return:
        '''
        self.running_mode = 0
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.th = None
        self.last_mode = 0
        self.thread_running = False

    def enter_srv_callback(self, _):
        rospy.loginfo("patrol enter")
        self.reset_value()
        return TriggerResponse(success=True)

    def exit_srv_callback(self, _):
        rospy.loginfo('patrol exit')
        self.reset_value()
        self.mecanum_pub.publish(geo_msg.Twist())
        return TriggerResponse(success=True)

    def set_running_srv_callback(self, req: SetInt64Request):
        '''
        设置模式(set the mode)
        :param req:
        :return:
        '''
        rsp = SetInt64Response(success=True)
        new_running_mode = req.data
        rospy.loginfo("set_running " + str(new_running_mode))
        if not 0 <= new_running_mode <= 4:
            rsp.success = False
            rsp.message = "Invalid running mode {}".format(new_running_mode)
            self.mecanum_pub.publish(geo_msg.Twist())
        else:
            # 以子线程模式运行，以允许停止(run under child thread mode to allow pause)
            if new_running_mode == 1:
                if self.th is None:
                    self.th = threading.Thread(target=self.rectangle)
                    self.th.start()
                else:
                    if not self.th.is_alive():
                        self.th = threading.Thread(target=self.rectangle)
                        self.th.start()
                    elif self.last_mode == new_running_mode:
                        pass
                    else:
                        self.thread_running = False
                        rospy.sleep(0.1)
                        self.rectangle()
            elif new_running_mode == 2:
                if self.th is None:
                    self.th = threading.Thread(target=self.triangle)
                    self.th.start()
                else:
                    if not self.th.is_alive():
                        self.th = threading.Thread(target=self.triangle)
                        self.th.start()
                    elif self.last_mode == new_running_mode:
                        pass
                    else:
                        self.thread_running = False
                        rospy.sleep(0.1)
                        self.triangle()
            elif new_running_mode == 3:
                if self.th is None:
                    self.th = threading.Thread(target=self.circle)
                    self.th.start()
                else:
                    if not self.th.is_alive():
                        self.th = threading.Thread(target=self.circle)
                        self.th.start()
                    elif self.last_mode == new_running_mode:
                        pass
                    else:
                        self.thread_running = False
                        rospy.sleep(0.1)
                        self.circle()
            elif new_running_mode == 4:
                if self.th is None:
                    self.th = threading.Thread(target=self.parallelogram)
                    self.th.start()
                else:
                    if not self.th.is_alive():
                        self.th = threading.Thread(target=self.parallelogram)
                        self.th.start()
                    elif self.last_mode == new_running_mode:
                        pass
                    else:
                        self.thread_running = False
                        rospy.sleep(0.1)
                        self.parallelogram()
            elif new_running_mode == 0:
                self.thread_running = False
                self.mecanum_pub.publish(geo_msg.Twist())
            self.running_mode = new_running_mode
            self.last_mode = self.running_mode
            return rsp

    def rectangle(self):
        # 走矩形
        status = 0
        t_start = rospy.get_time()
        self.thread_running = True
        while self.thread_running:
            current_time = rospy.get_time()
            if status == 0 and t_start < current_time:
                status = 1
                twist = geo_msg.Twist()
                twist.linear.x = self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_h / self.linear_speed
            elif status == 1 and t_start < current_time:
                status = 2
                twist = geo_msg.Twist()
                twist.linear.y = -self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_w / self.linear_speed
            elif status == 2 and t_start < current_time:
                status = 3
                twist = geo_msg.Twist()
                twist.linear.x = -self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_h / self.linear_speed
            elif status == 3 and t_start < current_time:
                status = 4
                twist = geo_msg.Twist()
                twist.linear.y = self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_w / self.linear_speed
            elif status == 4 and t_start < current_time:
                break

        self.mecanum_pub.publish(geo_msg.Twist())

    def parallelogram(self):
        # 走平行四边形(patrol along parallelogram)
        status = 0
        t_start = rospy.get_time()
        self.thread_running = True
        while self.thread_running:
            current_time = rospy.get_time()
            if status == 0 and t_start < current_time:
                status = 1
                twist = geo_msg.Twist()
                twist.linear.x = self.linear_speed * math.cos(math.pi / 6)
                twist.linear.y = -self.linear_speed * math.sin(math.pi / 6)
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_w / self.linear_speed
            elif status == 1 and t_start < current_time:
                status = 2
                twist = geo_msg.Twist()
                twist.linear.y = -self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_h / self.linear_speed
            elif status == 2 and t_start < current_time:
                status = 3
                twist = geo_msg.Twist()
                twist.linear.x = -self.linear_speed * math.cos(math.pi / 6)
                twist.linear.y = self.linear_speed * math.sin(math.pi / 6)
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_w / self.linear_speed
            elif status == 3 and t_start < current_time:
                status = 4
                twist = geo_msg.Twist()
                twist.linear.y = self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.rectangle_h / self.linear_speed
            elif status == 4 and t_start < current_time:
                break

        self.mecanum_pub.publish(geo_msg.Twist())

    def triangle(self):
        # 走三角 (patrol along triangle)
        status = 0
        t_start = rospy.get_time()
        self.thread_running = True
        while self.thread_running:
            current_time = rospy.get_time()
            if status == 0 and t_start < current_time:
                status = 1
                twist = geo_msg.Twist()
                twist.linear.x = self.linear_speed * math.cos(math.pi / 6)
                twist.linear.y = -self.linear_speed * math.sin(math.pi / 6)
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.triangle_l / self.linear_speed
            elif status == 1 and t_start < current_time:
                status = 2
                twist = geo_msg.Twist()
                twist.linear.x = -self.linear_speed * math.cos(math.pi / 6)
                twist.linear.y = -self.linear_speed * math.sin(math.pi / 6)
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.triangle_l / self.linear_speed
            elif status == 2 and t_start < current_time:
                status = 3
                twist = geo_msg.Twist()
                twist.linear.y = self.linear_speed
                self.mecanum_pub.publish(twist)
                t_start = current_time + self.triangle_l / self.linear_speed
            elif status == 3 and t_start < current_time:
                break

        self.mecanum_pub.publish(geo_msg.Twist())

    def circle(self):
        # 走圆形(patrol along circle)
        status = 0
        t_start = rospy.get_time()
        self.thread_running = True
        while self.thread_running:
            current_time = rospy.get_time()
            if status == 0 and t_start < current_time:
                status = 1
                twist = geo_msg.Twist()
                twist.linear.x = self.linear_speed
                twist.angular.z = -0.5
                self.mecanum_pub.publish(twist)
                t_start = current_time + 2 * math.pi / 0.5
            elif status == 1 and t_start < current_time:
                break

        self.mecanum_pub.publish(geo_msg.Twist())


if __name__ == "__main__":
    PatrolController('patrol_app')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
