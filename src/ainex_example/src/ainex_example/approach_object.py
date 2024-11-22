#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/04
# @author:aiden
# 接近物体逻辑处理
import math
from ainex_sdk import misc

class ApproachObject:
    def __init__(self, gait_manager, x_stop=0.78, y_stop=0.5, yaw_stop=0, debug=False):
        self.debug = debug
        self.count_stop = 0
        self.x_stop = x_stop  # 躯体停止前进时物体在画面的y轴上的比值(如画面为160x120，则物体的纵坐标为120x0.75)
        self.y_stop = y_stop  
        self.yaw_stop = yaw_stop  # 躯体停止转动时线的角度
        self.x_range = [-0.01, 0.01]  # 前进脚步大小单位m
        self.y_range = [-0.015, 0.015]  # 前进脚步大小单位m
        self.yaw_range = [-5, 5]  # 转弯范围正负10度
        self.gait_manager = gait_manager
        
        self.gait_param = self.gait_manager.get_gait_param()
        self.gait_param['pelvis_offset'] = 5
        self.gait_param['step_height'] = 0.02
        self.gait_param['z_swap_amplitude'] = 0.006
        self.dsp = [400, 0.1, 0.03]
        self.arm_swap = 30

    def update_gait(self, dsp=[400, 0.1, 0.03], arm_swap=30, walking_param=None):
        if walking_param is not None:
            self.gait_param = walking_param
        self.dsp = dsp
        self.arm_swap = arm_swap

    def process(self, center_x, max_y, angle, width, height):

        # 根据线的倾斜角度进行旋转调整
        if abs(angle - self.yaw_stop) < 10:
            yaw_output = misc.val_map(angle - self.yaw_stop, -10, 10, self.yaw_range[0], self.yaw_range[1])
        else:
            yaw_output = math.copysign(self.yaw_range[1], angle)
        if abs(yaw_output) < 3:
            yaw_output = math.copysign(3, yaw_output)  

        y_stop = self.y_stop*width
        if abs(y_stop - center_x) < 80:
            y_output = misc.val_map(y_stop - center_x, -80, 80, self.y_range[0], self.y_range[1])
        else:
            y_output = math.copysign(self.y_range[1], y_stop - center_x)
        if abs(y_output) < 0.005:
            y_output = math.copysign(0.005, y_output)      

        # 前进根据头部上下舵机位置
        x_stop = self.x_stop*height
        if abs(max_y - x_stop) > height / 4:
            x_output = math.copysign(self.x_range[1], x_stop - max_y)
        else:
            x_output = misc.val_map(x_stop - max_y, -height / 4, height / 4, self.x_range[0], self.x_range[1])
        if abs(x_output) < 0.005:
            x_output = math.copysign(0.005, x_output)

        # print(x_output, y_output, yaw_output, center_x, max_y, angle)
        if self.debug: 
            print(center_x, max_y, angle)
        else:
            if abs(angle) >= 5 or abs(max_y - x_stop) >= 40 or abs(y_stop - center_x) >= 30:
                self.count_stop = 0
                self.gait_manager.set_step(self.dsp, round(x_output, 4), round(y_output, 4), -int(yaw_output), self.gait_param, arm_swap=self.arm_swap, step_num=0)
            else:
                self.count_stop += 1
                self.gait_manager.stop()  # 需要先关闭行走
        if self.count_stop > 1:
            print('stop')
            self.count_stop = 0
            return True
        else:
            return False
