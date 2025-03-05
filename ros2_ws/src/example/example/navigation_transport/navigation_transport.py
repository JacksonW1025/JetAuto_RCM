#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/18
# @author:aiden
# 导航搬运
import math
import rclpy
import numpy as np
from rclpy.node import Node
import sdk.common as common
from std_srvs.srv import Trigger
from rclpy.duration import Duration
from interfaces.srv import SetPose2D
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from rclpy.executors import MultiThreadedExecutor
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class NavigationTransport(Node):
    markerArray = MarkerArray()
    
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self.pick = True
        self.place = False
        self.running = True
        self.goal_pose = PoseStamped()
        self.haved_publish_goal = False
        self.navigator = BasicNavigator()

        self.map_frame = self.get_parameter('map_frame').value
        self.nav_goal = self.get_parameter('nav_goal').value

        timer_cb_group = ReentrantCallbackGroup()
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.nav_pub = self.create_publisher(PoseStamped, self.nav_goal, 1)
        self.mark_pub = self.create_publisher(MarkerArray, 'path_point', 1)
        
        self.create_subscription(PoseStamped, self.nav_goal, self.goal_callback, 1, callback_group=timer_cb_group)

        self.create_service(SetPose2D, '~/pick', self.start_pick_srv_callback)
        self.create_service(SetPose2D, '~/place', self.start_place_srv_callback)
       
        self.pick_client = self.create_client(Trigger, '/automatic_pick/pick', callback_group=timer_cb_group)
        self.place_client = self.create_client(Trigger, '/automatic_pick/place', callback_group=timer_cb_group)

        self.pick_client.wait_for_service()
        self.place_client.wait_for_service()

        self.get_param_client = self.create_client(GetParameters, '/automatic_pick/get_parameters', callback_group=timer_cb_group)
        self.get_param_client.wait_for_service()
        
        self.navigator.waitUntilNav2Active()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def set_parameter(self, client, name, value):
        # Parameter.Type.INTEGER、Parameter.Type.DOUBLE、Parameter.Type.STRING、Parameter.Type.BOOLEAN、Parameter.Type.BYTE_ARRA
        req = SetParametersAtomically.Request()
        req.parameters = [Parameter(name, Parameter.Type.STRING, value).to_parameter_msg()]
        client.call_async(req)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def start_pick_srv_callback(self, request, response):
        self.get_logger().info('start navigaiton pick')

        marker_Array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.action = Marker.DELETEALL
        marker_Array.markers.append(marker)

        self.mark_pub.publish(marker_Array)

        markerArray = MarkerArray()
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        data = request.data
        q = common.rpy2qua(math.radians(data.roll), math.radians(data.pitch), math.radians(data.yaw))
        pose.pose.position.x = data.x
        pose.pose.position.y = data.y
        pose.pose.orientation = q

        # 用数字标记来显示点(mark the point with number to display)
        marker = Marker()
        marker.header.frame_id = self.map_frame

        marker.type = marker.MESH_RESOURCE
        marker.mesh_resource = "package://example/resource/flag.dae"
        marker.action = marker.ADD
        # 大小(size)
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.2
        # 颜色(color)
        color = list(np.random.choice(range(256), size=3))
        marker.color.a = 1.0
        marker.color.r = color[0] / 255.0
        marker.color.g = color[1] / 255.0
        marker.color.b = color[2] / 255.0
        # marker.lifetime = rospy.Duration(10)  # 显示时间，没有设置默认一直保留(display time. If not set, it will be kept by default)
        # 位置姿态
        marker.pose.position.x = pose.pose.position.x
        marker.pose.position.y = pose.pose.position.y
        marker.pose.orientation = pose.pose.orientation
        markerArray.markers.append(marker)

        self.mark_pub.publish(markerArray)
        self.nav_pub.publish(pose)
        
        response.success = True
        response.message = "navigation pick"
        return response

    def start_place_srv_callback(self, request, response):
        self.get_logger().info('start navigaiton place')

        markerArray = MarkerArray()
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        data = request.data
        q = common.rpy2qua(math.radians(data.roll), math.radians(data.pitch), math.radians(data.yaw))
        pose.pose.position.x = data.x
        pose.pose.position.y = data.y
        pose.pose.orientation = q

        # 用数字标记来显示点(mark the point with number to display)
        marker = Marker()
        marker.header.frame_id = self.map_frame

        marker.type = marker.MESH_RESOURCE
        marker.mesh_resource = "package://example/resource/flag.dae"
        marker.action = marker.ADD
        # 大小(size)
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.2
        # 颜色(color)
        color = list(np.random.choice(range(256), size=3))
        marker.color.a = 1.0
        marker.color.r = color[0] / 255.0
        marker.color.g = color[1] / 255.0
        marker.color.b = color[2] / 255.0
        # marker.lifetime = rospy.Duration(10)  # 显示时间，没有设置默认一直保留(display time. If not set, it will be kept by default)
        # 位置姿态
        marker.pose.position.x = pose.pose.position.x
        marker.pose.position.y = pose.pose.position.y
        marker.pose.orientation = pose.pose.orientation
        markerArray.markers.append(marker)

        self.mark_pub.publish(markerArray)
        self.nav_pub.publish(pose)

        response.success = True
        response.message = "navigation place"
        return response

    def goal_callback(self, msg):
        # 获取要发布的导航点
        self.get_logger().info('\033[1;32m%s\033[0m' % str(msg))

        get_parameters_request = GetParameters.Request()
        get_parameters_request.names = ['status']
        status = self.send_request(self.get_param_client, get_parameters_request).values[0].string_value
        self.get_logger().info('\033[1;32m%s\033[0m' % status)
        if status == 'start' or status == 'place_finish':  # 处于可以pick的状态
            self.pick = True
            self.place = False
            self.get_logger().info('\033[1;32m%s\033[0m' % 'nav pick')

            self.navigator.goToPose(msg)
            self.haved_publish_goal = True
        elif status == 'pick_finish':  # 处于可以place的状态
            self.pick = False
            self.place = True
            self.get_logger().info('\033[1;32m%s\033[0m' % 'nav place')

            self.navigator.goToPose(msg)
            self.haved_publish_goal = True

        if self.haved_publish_goal:
            i = 0
            while not self.navigator.isTaskComplete():
                i = i + 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 5 == 0:
                    self.get_logger().info(
                        'Estimated time of arrival: '
                        + '{0:.0f}'.format(
                            Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                            / 1e9
                        )
                        + ' seconds.'
                    )

                    # Some navigation timeout to demo cancellation
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        self.navigator.cancelTask()

                    # Some navigation request change to demo preemption
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                        self.goal_pub.publish(self.goal_pose)
                self.get_logger().info('\033[1;32m%s\033[0m' % 'feedback')
            # Do something depending on the return code
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Goal succeeded!')
                if self.pick:
                    res = self.send_request(self.pick_client, Trigger.Request())
                    if res.success:
                        self.get_logger().info('start pick')
                    else:
                        self.get_logger().info('start pick failed')
                else:
                    res = self.send_request(self.place_client, Trigger.Request())
                    if res.success:
                        self.get_logger().info('start place')
                    else:
                        self.get_logger().info('start place failed')

                self.haved_publish_goal = False
            elif result == TaskResult.CANCELED:
                self.get_logger().info('Goal was canceled!')
            elif result == TaskResult.FAILED:
                self.get_logger().info('Goal failed!')
            else:
                self.get_logger().info('Goal has an invalid return status!')

def main():
    node = NavigationTransport('navigation_transport')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()


