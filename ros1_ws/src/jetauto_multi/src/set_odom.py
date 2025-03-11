#!/usr/bin/env python
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/11/08
import os
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

rospy.init_node('set_odom')
robot_name = rospy.get_param('~robot_name', 'robot_2')
rospy.wait_for_message(robot_name + "/odom", Odometry)
slam_methods = rospy.get_param('~slam_methods')
x = rospy.get_param('~x')
y = rospy.get_param('~y')
odom_pub = rospy.Publisher(robot_name + '/set_odom', Pose2D, queue_size=1)
pose = Pose2D()
pose.x = x 
pose.y = y
rospy.sleep(0.2)
odom_pub.publish(pose)
os.system("roslaunch /home/jetauto/jetauto_ws/src/jetauto_slam/launch/include/slam_base.launch slam_methods:=" + slam_methods)