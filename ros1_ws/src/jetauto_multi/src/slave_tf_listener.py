#!/usr/bin/env python
# encoding: utf-8
import tf
import math
import rospy
import jetauto_sdk.pid as pid
from geometry_msgs.msg import Twist


def qua2rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))

    return roll, pitch, yaw


if __name__ == '__main__':
    rospy.init_node('tf_listener')

    # TransformListener创建后就开始接受tf广播信息，最多可以缓存10s(after TransformListener
    # is created, start receiving tf broadcast message. The maximum cache size is 10s)
    listener = tf.TransformListener()

    cmd_vel = rospy.get_param('~cmd_vel', '/robot_2/robot_controller/cmd_vel')
    base_frame = rospy.get_param('~base_frame', '/robot_2/base_footprint')
    target_frame = rospy.get_param('~target_frame', '/point2')

    robot_vel = rospy.Publisher(cmd_vel, Twist, queue_size=1)
    rate = rospy.Rate(30.0)  # 循环执行，更新频率是10hz(execute in loop. The update frequency is 10hz)

    pid_x = pid.PID(1.5, 0, 0)
    pid_y = pid.PID(1.5, 0, 0)
    pid_z = pid.PID(0.1, 0, 0)

    while not rospy.is_shutdown():
        msg = Twist()
        try:
            # 查看相对的tf, 返回平移和旋转(check relative tf, and return translation and rotation)
            (trans, rot) = listener.lookupTransform(base_frame, target_frame, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            robot_vel.publish(msg)
            rate.sleep()
            continue

        # print(trans, rot)

        x = trans[0]
        y = trans[1]
        angle = math.degrees(qua2rpy(rot[0], rot[1], rot[2], rot[3])[-1])

        pid_x.SetPoint = 0
        if abs(x) < 0.03:
            x = 0
        pid_x.update(x)  # 更新pid(update pid)
        linear_x = -pid_x.output

        pid_y.SetPoint = 0
        if abs(y) < 0.03:
            y = 0
        pid_y.update(y)  # 更新pid(update pid)
        linear_y = -pid_y.output

        pid_z.SetPoint = 0
        if abs(angle) < 5:
            angle = 0
        pid_z.update(angle)  # 更新pid(update pid)
        angular_z = -pid_z.output

        if linear_x > 0.25:
            linear_x = 0.25
        if linear_x < -0.25:
            linear_x = -0.25
        if linear_y > 0.25:
            linear_y = 0.25
        if linear_y < -0.25:
            linear_y = -0.25
        if angular_z > 0.4:
            angular_z = 0.4
        if angular_z < -0.4:
            angular_z = -0.4

        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z

        robot_vel.publish(msg)
        rate.sleep()  # 以固定频率执行(execute at the fixed frequency)
