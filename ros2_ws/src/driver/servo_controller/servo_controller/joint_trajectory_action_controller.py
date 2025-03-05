#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2023/11/10
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory

class Segment:
    def __init__(self, num_joints):
        self.start_time = 0.0  # trajectory segment start time
        self.duration = 0.0  # trajectory segment duration
        self.positions = [0.0] * num_joints
        self.velocities = [0.0] * num_joints

class JointTrajectoryActionController(Node):
    def __init__(self, servo_manager, controller_namespace, controllers):
        super().__init__(controller_namespace)
        self.servo_manager = servo_manager
        
        self.joint_names = []
        self.joint_to_controller = {}
        for c in controllers:
            self.joint_names.append(c.joint_name)
            self.joint_to_controller[c.joint_name] = c
            # self.get_logger().info(self.joint_names)
        self.num_joints = len(self.joint_names)

        self.goal_constraints = []
        self.trajectory_constraints = []
        ns = controller_namespace + '/joint_trajectory_action_node/constraints'
        for joint in self.joint_names:
            self.goal_constraints.append(-1)
            self.trajectory_constraints.append(-1)

        # Message containing current state for all controlled joints
        self.feedback_msg = FollowJointTrajectory.Feedback()
        self.feedback_msg.joint_names = self.joint_names
        self.feedback_msg.desired.positions = [0.0] * self.num_joints
        self.feedback_msg.desired.velocities = [0.0] * self.num_joints
        self.feedback_msg.desired.accelerations = [0.0] * self.num_joints
        self.feedback_msg.actual.positions = [0.0] * self.num_joints
        self.feedback_msg.actual.velocities = [0.0] * self.num_joints
        self.feedback_msg.error.positions = [0.0] * self.num_joints
        self.feedback_msg.error.velocities = [0.0] * self.num_joints
        self.action_server = ActionServer(self, FollowJointTrajectory, controller_namespace + '/follow_joint_trajectory', self.follow_trajectory_callback)

    def wait_action(self):
        if self.action_server.is_active():
            self.action_server.set_preempted()

        while self.action_server.is_active():
            time.sleep(0.01)

    def follow_trajectory_callback(self, goal_handle):
        goal = goal_handle.request
        traj = goal.trajectory
        num_points = len(traj.points)  # 计算总的轨迹点数

        if num_points == 0:  # 如果没有轨迹点则立刻返回
            msg = 'Incoming trajectory is empty'
            self.get_logger().error(msg)
            goal_handle.abort()
            return

        lookup = []
        for joint in self.joint_names:
            lookup.append(traj.joint_names.index(joint))  # 将joint的顺序转为数字索引
        durations = [0.0] * num_points

        # find out the duration of each segment in the trajectory
        durations[0] = traj.points[0].time_from_start.sec  # 第一个点的时间戳

        for i in range(1, num_points):
            # 下一个减去上一个的时间戳算出来就是这个轨迹运行需要的时间
            durations[i] = (traj.points[i].time_from_start - traj.points[i - 1].time_from_start).sec

        if not traj.points[0].positions:  # 如果为空
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            msg = 'First point of trajectory has no positions'
            self.get_logger().error(msg)
            goal_handle.abort()
            return

        trajectory = []
        current_time = self.get_clock().now() + Duration(seconds=0.01)

        for i in range(num_points):  # 遍历所有轨迹点，将他重新存储到列表里
            seg = Segment(self.num_joints)

            if traj.header.stamp == Time(0.0):
                seg.start_time = (current_time + traj.points[i].time_from_start).seconds - durations[i]
            else:
                seg.start_time = (traj.header.stamp + traj.points[i].time_from_start).seconds - durations[i]

            seg.duration = durations[i]

            for j in range(self.num_joints):
                if traj.points[i].positions:
                    seg.positions[j] = traj.points[i].positions[lookup[j]]

            trajectory.append(seg)

        self.get_logger().info('Trajectory start requested at %.3lf, waiting...', traj.header.stamp.sec)

        while traj.header.stamp > current_time:
            current_time = self.get_clock().now()
            time.sleep(0.001)

        end_time = traj.header.stamp + Duration(seconds=sum(durations))
        seg_end_times = [Time(seconds=trajectory[seg].start_time + durations[seg]) for seg in
                         range(len(trajectory))]

        self.get_logger().info('Trajectory start time is %.3lf, end time is %.3lf, total duration is %.3lf',
                               current_time.seconds,
                               end_time.seconds, sum(durations))

        for seg in range(len(trajectory)):
            self.get_logger().debug('current segment is %d time left %f cur time %f' % (
                seg, durations[seg] - (current_time.seconds - trajectory[seg].start_time), current_time.seconds))
            self.get_logger().debug('goal positions are: %s' % str(trajectory[seg].positions))

            # first point in trajectories calculated by OMPL is current position with duration of 0 seconds, skip it
            if durations[seg] == 0:
                self.get_logger().debug('skipping segment %d with duration of 0 seconds' % seg)
                continue

            position = []
            for joint in self.self.joint_names:
                j = self.joint_names.index(joint)
                desired_position = trajectory[seg].positions[j]
                self.feedback_msg.desired.positions[j] = desired_position
                servo_id = self.joint_to_controller[joint].servo_id
                pos = self.joint_to_controller[joint].pos_rad_to_raw(desired_position)
                position.append((servo_id, pos))

            for id_, pos_ in position:
                self.servo_manager.set_position(id_, pos_, durations[seg])

            while current_time < seg_end_times[seg]:
                # heck if new trajectory was received, if so abort current trajectory execution
                # by setting the goal to the current position c
                if goal_handle.is_cancel_requested:
                    msg = 'New trajectory received. Exiting.'
                    self.get_logger().info(msg)
                    goal_handle.abort()
                    return

                time.sleep(0.001)
                current_time = self.get_clock().now()

            # Verifies trajectory constraints
            for j, joint in enumerate(self.joint_names):
                if self.trajectory_constraints[j] > 0 and self.feedback_msg.error.positions[j] > self.trajectory_constraints[j]:
                    res = FollowJointTrajectory.Result()
                    res.error_code = FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED
                    msg = 'Unsatisfied position constraint for %s, trajectory point %d, %f is larger than %f' % \
                          (joint, seg, self.feedback_msg.error.positions[j], self.trajectory_constraints[j])
                    self.get_logger().warn(msg)
                    goal_handle.abort()
                    return

        # Checks that we have ended inside the goal constraints
        for (joint, pos_error, pos_constraint) in zip(self.joint_names, self.feedback_msg.error.positions,
                                                      self.goal_constraints):
            if pos_constraint > 0 and abs(pos_error) > pos_constraint:
                res = FollowJointTrajectory.Result()
                res.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                msg = 'Aborting because %s joint wound up outside the goal constraints, %f is larger than %f' % \
                      (joint, pos_error, pos_constraint)
                self.get_logger().warn(msg)
                goal_handle.abort()
                break
        else:
            msg = 'Trajectory execution successfully completed'
            self.get_logger().info(msg)
            res = FollowJointTrajectoryResult()
            res.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            goal_handle.succeed()
