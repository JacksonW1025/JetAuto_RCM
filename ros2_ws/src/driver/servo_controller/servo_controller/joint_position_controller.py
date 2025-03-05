#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2023/11/10
import math
# from rclpy.node import Node

class JointPositionController():
    def __init__(self, joint_config, joint_name):
        # super().__init__(joint_name)
        self.RADIANS_PER_ENCODER_TICK = 240 / 360 * (math.pi * 2) / 1000  # 脉宽--->弧度
        self.ENCODER_TICKS_PER_RADIAN = 1 / self.RADIANS_PER_ENCODER_TICK # 弧度-->脉宽
        self.ENCODER_RESOLUTION = 1000
        self.MAX_POSITION = self.ENCODER_RESOLUTION - 1
        self.VELOCITY_PER_TICK = 10
        self.MAX_VELOCITY = 100
        self.MIN_VELOCITY = self.VELOCITY_PER_TICK

        self.joint_name = joint_name
        self.servo_id = int(joint_config['id'].value)
        self.initial_position_raw = int(joint_config['init'].value)
        self.min_angle_raw = int(joint_config['min'].value)
        self.max_angle_raw = int(joint_config['max'].value)

        self.flipped = self.min_angle_raw > self.max_angle_raw
        if self.flipped:
            self.min_angle = (self.initial_position_raw - self.min_angle_raw) * self.RADIANS_PER_ENCODER_TICK
            self.max_angle = (self.initial_position_raw - self.max_angle_raw) * self.RADIANS_PER_ENCODER_TICK
        else:
            self.min_angle = (self.min_angle_raw - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK
            self.max_angle = (self.max_angle_raw - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK

        self.min_pulse = min(self.min_angle_raw, self.max_angle_raw)
        self.max_pulse = max(self.min_angle_raw, self.max_angle_raw)

    def rad_to_pulse(self, angle, initial_position_raw, flipped, encoder_ticks_per_radian):
        angle_raw = angle * encoder_ticks_per_radian
        return initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw

    def pulse_to_rad(self, raw, initial_position_raw, flipped, radians_per_encoder_tick):
        return (initial_position_raw - raw if flipped else raw - initial_position_raw) * radians_per_encoder_tick

    def pos_rad_to_pulse(self, pos_rad):  #弧度--->脉宽
        if pos_rad < self.min_angle:
            pos_rad = self.min_angle
        elif pos_rad > self.max_angle:
            pos_rad = self.max_angle
        return self.rad_to_pulse(pos_rad, self.initial_position_raw, self.flipped, self.ENCODER_TICKS_PER_RADIAN)

    def pos_pulse_to_rad(self, pos_pulse):  #脉宽--->弧度
        if pos_pulse < self.min_pulse:
            pos_pulse = self.min_pulse
        elif pos_pulse > self.max_pulse:
            pos_pulse = self.max_pulse
        return self.pulse_to_rad(pos_pulse, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)
