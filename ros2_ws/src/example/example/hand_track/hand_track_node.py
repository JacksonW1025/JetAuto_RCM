#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 手跟随
import time
import rclpy
import signal
import threading
import sdk.pid as pid
from rclpy.node import Node
from std_srvs.srv import Trigger
from kinematics import transform
from interfaces.msg import Point2D
from geometry_msgs.msg import Twist
from kinematics_msgs.srv import SetRobotPose
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from kinematics.kinematics_control import set_pose_target
from servo_controller.bus_servo_control import set_servo_position

class HandTrackNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.name = name
        self.image = None
        self.center = None
        self.running = True
        self.z_dis = 0.41
        self.y_dis = 500
        self.x_init = transform.link3 + transform.tool_link

        self.pid_z = pid.PID(0.00005, 0.0, 0.0)
        self.pid_y = pid.PID(0.04, 0.0, 0.0)

        signal.signal(signal.SIGINT, self.shutdown)

        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) # 舵机控制

        self.image_sub = self.create_subscription(Point2D, '/hand_detect/center', self.get_hand_callback, 1)
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

    def shutdown(self, signum, frame):
        self.running = False

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

    def get_hand_callback(self, msg):
        if msg.width != 0:
            self.center = msg
        else:
            self.center = None

    def main(self):
        while self.running:
            if self.center is not None:
                t1 = time.time()
                self.pid_y.SetPoint = self.center.width / 2
                self.pid_y.update(self.center.width - self.center.x)
                self.y_dis += self.pid_y.output
                if self.y_dis < 200:
                    self.y_dis = 200
                if self.y_dis > 800:
                    self.y_dis = 800

                self.pid_z.SetPoint = self.center.height / 2
                self.pid_z.update(self.center.y)
                self.z_dis += self.pid_z.output
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
                else:
                    set_servo_position(self.joints_pub, 0.02, ((1, int(self.y_dis)), ))

            else:
                time.sleep(0.01)

        self.init_action()
        rclpy.shutdown()

def main():
    node = HandTrackNode('hand_track')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()

