#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from servo_controller import action_group_controller
from servo_controller_msgs.msg import ServosPosition, ServoPosition

class InitPose(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)  # 允许未声明的参数
        
        namespace = self.get_namespace()
        if namespace == '/':
            namespace = ''
       
        self.servo_controller_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.joint_controller_pub = self.create_publisher(JointState, 'joint_controller', 1)

        self.client = self.create_client(Trigger, namespace + '/controller_manager/init_finish')
        self.client.wait_for_service()

        # 读取配置参数
        self.type = self.get_parameter('type').value
        if self.type == 'action':
            action = self.get_parameters_by_prefix(self.type)
            acg = action_group_controller.ActionGroupController(self.servo_controller_pub, '/home/ubuntu/share/arm_pc/ActionGroups')
            acg.run_action(action['action_name'].value)
        elif self.type == 'servo':
            pulse = self.get_parameters_by_prefix(self.type)
            msg = ServosPosition()
            msg.duration = float(pulse['duration'].value)
            data = []   
            for i in ['id1', 'id2', 'id3', 'id4', 'id5', 'id10']:
                servo = ServoPosition()
                servo.id = int(i[2:]) 
                servo.position = int(pulse[i].value)
                data.append(servo)
            msg.position = data
            self.servo_controller_pub.publish(msg)

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

def main():
    node = InitPose('init_pose')
    rclpy.spin(node)  # 循环等待ROS2退出

if __name__ == "__main__":
    main()
