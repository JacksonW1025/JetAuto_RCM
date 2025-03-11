#!/usr/bin/env python
# encoding: utf-8
# @aiden
# @2022/11/08
import tf
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger, TriggerResponse

x2, x3, y2, y3 = 0, 0, 0, 0
def set_row_srv(msg):
    global x2, x3, y2, y3
    x2 = 0
    x3 = 0
    y2 = -0.7
    y3 = 0.7
    return TriggerResponse(success=True)

def set_column_srv(msg):
    global x2, x3, y2, y3
    x2 = -0.6
    x3 = -1.2
    y2 = 0
    y3 = 0
    return TriggerResponse(success=True)

def set_triangle_srv(msg):
    global x2, x3, y2, y3
    x2 = -0.7
    x3 = -0.7
    y2 = -0.4
    y3 = 0.4
    return TriggerResponse(success=True)

if __name__ == "__main__":
    # 初始化 ROS 节点
    rospy.init_node("static_tf_publish")
    master_name = rospy.get_param("~master_name")
    multi_mode = rospy.get_param("~multi_mode")
    if multi_mode == 'row':
        set_row_srv(Trigger())
    elif multi_mode == 'column':
        set_column_srv(Trigger())
    else:
        set_triangle_srv(Trigger())

    rospy.Service('/set_row', Trigger, set_row_srv)
    rospy.Service('/set_column', Trigger, set_column_srv)
    rospy.Service('/set_triangle', Trigger, set_triangle_srv)

    # 创建静态坐标广播器
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    # 创建并组织被广播的消息
    robot_2 = TransformStamped()
    # 头信息
    robot_2.header.frame_id = master_name + "/base_footprint"
    # 子坐标系
    robot_2.child_frame_id = "point2"
    robot_2.transform.translation.z = 0
    # 四元数
    qtn = tf.transformations.quaternion_from_euler(0, 0, 0)
    robot_2.transform.rotation.x = qtn[0]
    robot_2.transform.rotation.y = qtn[1]
    robot_2.transform.rotation.z = qtn[2]
    robot_2.transform.rotation.w = qtn[3]

    # 创建并组织被广播的消息
    robot_3 = TransformStamped()
    # 头信息
    robot_3.header.frame_id = master_name + "/base_footprint"
    # 子坐标系
    robot_3.child_frame_id = "point3"
    robot_3.transform.translation.z = 0
    # 四元数
    qtn = tf.transformations.quaternion_from_euler(0, 0, 0)
    robot_3.transform.rotation.x = qtn[0]
    robot_3.transform.rotation.y = qtn[1]
    robot_3.transform.rotation.z = qtn[2]
    robot_3.transform.rotation.w = qtn[3]

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        # 头信息
        robot_2.header.stamp = rospy.Time.now()
        # 坐标系相对信息
        # 偏移量
        robot_2.transform.translation.x = x2
        robot_2.transform.translation.y = y2

        # 头信息
        robot_3.header.stamp = rospy.Time.now()
        # 坐标系相对信息
        # 偏移量
        robot_3.transform.translation.x = x3
        robot_3.transform.translation.y = y3
        # 广播器发送消息
        broadcaster.sendTransform(robot_2)
        broadcaster.sendTransform(robot_3)
        rate.sleep()  # 以固定频率执行
