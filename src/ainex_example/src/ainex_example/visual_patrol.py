#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/06
# @author:aiden
# 巡线逻辑处理
import math
from ainex_sdk import misc

class VisualPatrol:
    def __init__(self, gait_manager):
        self.x_range = [0, 0.02]  # 前进脚步大小单位m
        self.yaw_range = [-10, 10]  # 转弯范围正负10度
        self.gait_manager = gait_manager
        self.go_gait_param = self.gait_manager.get_gait_param()
        self.go_gait_param['body_height'] = 0.015
        self.go_gait_param['step_height'] = 0.02
        self.go_gait_param['pelvis_offset'] = 5
        self.go_gait_param['hip_pitch_offset'] = 20
        self.go_gait_param['z_swap_amplitude'] = 0.006
        self.go_dsp = [150, 0.25, 0.01]
        self.go_arm_sawp = 30

        self.turn_gait_param = self.gait_manager.get_gait_param()
        self.turn_gait_param['body_height'] = 0.015
        self.turn_gait_param['step_height'] = 0.02
        self.turn_gait_param['pelvis_offset'] = 5
        self.turn_gait_param['hip_pitch_offset'] = 20
        self.turn_gait_param['z_swap_amplitude'] = 0.006
        self.turn_dsp = [150, 0.25, 0.01]
        self.turn_arm_swap = 30

        self.x_max = 0.02

    def update_turn_gait(self, dsp=[400, 0.2, 0.02], x_max=0.02, arm_swap=30, walking_param=None):
        if walking_param is not None:
            self.turn_gait_param = walking_param
        self.turn_dsp = dsp
        self.turn_arm_swap = arm_swap
        self.x_max = x_max

    def update_go_gait(self, dsp=[300, 0.2, 0.02], x_max=0.02, arm_swap=30, walking_param=None):
        if walking_param is not None:
            self.go_gait_param = walking_param
        self.go_dsp = dsp
        self.go_arm_swap = arm_swap
        self.x_max = x_max

    def process(self, x, width):
        # 左右根据x坐标值进行调整
        if abs(x - width/2) < 10:
            yaw_output = 0
        elif abs(x - width/2) < width/4:
            yaw_output = math.copysign(1, x - width/2) + misc.val_map(x - width/2, -width/4, width/4, self.yaw_range[0] + 1, self.yaw_range[1] - 1)
        else:
            yaw_output = math.copysign(self.yaw_range[1], x - width/2)
        
        # 转弯时前进步幅减小
        if abs(yaw_output) > 7:
            x_output = 0.015
        else:
            x_output = self.x_max
        if abs(yaw_output) < 5:
            self.gait_manager.set_step(self.go_dsp, x_output, 0, int(-yaw_output), self.go_gait_param, arm_swap=self.go_arm_sawp, step_num=0)
        else:
            self.gait_manager.set_step(self.turn_dsp, x_output, 0, int(-yaw_output), self.turn_gait_param, arm_swap=self.turn_arm_swap, step_num=0)
