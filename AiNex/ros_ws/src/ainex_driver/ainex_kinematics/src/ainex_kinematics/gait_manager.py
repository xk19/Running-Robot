#!/usr/bin/env python3
# encoding: utf-8
# Date:2023/06/16
import math
import rospy
from ainex_interfaces.msg import WalkingParam
from ainex_interfaces.srv import GetWalkingParam, SetWalkingCommand

class ErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const

    def __str__(self):
        return self.message

class GaitManager:
    def __init__(self):
        self.have_start = False
        # period_time, dsp_ratio, y_swap_amplitude
        # 步态周期ms, 双脚触地占比(0-1), 摆动幅度m
        self.dsp_ratio = [[200, 0.3, 0.02],
                          [300, 0.3, 0.02],
                          [400, 0.3, 0.02],
                          [500, 0.3, 0.02]
                          ]

        # 躯体可以比完全站直低的范围，单位m
        self.body_height_range = [0.015, 0.06] 
        # 步幅范围，单位m
        self.x_amplitude_range = [0.0, 0.02] 
        self.y_amplitude_range = [0.0, 0.02]
        # 腿抬高范围，单位m 
        self.step_height_range = [0.01, 0.04]
        # 转弯范围，单位度
        self.rotation_angle_range = [0, 10]
        # 手臂摆动范围，单位度
        self.arm_swap_range = [0, 60]

        self.y_swap_range = [0, 0.05]
        self.dsp_ratio_range = [0, 1]

        rospy.wait_for_service('/walking/get_param')
        res = rospy.ServiceProxy('/walking/get_param', GetWalkingParam)()
        self.walking_param = res.parameters
        self.param_pub = rospy.Publisher('/walking/set_param', WalkingParam, queue_size=1)

        rospy.sleep(0.2)

    def update_param(self, step_velocity, x_amplitude, y_amplitude, rotation_angle, body_height=0.015, step_height=0.025,
                 pelvis_offset=3, hip_pitch_offset=20, z_swap_amplitude=0.01, arm_swap=30):
        if step_velocity[0] < 0:
            raise Exception('period_time cannot be negative' % step_velocity[0])
        if step_velocity[1] > self.dsp_ratio_range[1] or step_velocity[1] < self.dsp_ratio_range[0]:
            raise Exception('dsp_ratio_range %d out of range(0~1)' % step_velocity[1])
        if step_velocity[2] > self.y_swap_range[1] or step_velocity[2] < self.y_swap_range[0]:
            raise Exception('y_swap_range %d out of range(0~0.05)' % step_velocity[2])
        if abs(x_amplitude) > self.x_amplitude_range[1] or abs(x_amplitude) < self.x_amplitude_range[0]:
            raise Exception('x_amplitude %d out of range(-0.02~0.02)' % x_amplitude)
        if abs(y_amplitude) > self.y_amplitude_range[1] or abs(y_amplitude) < self.y_amplitude_range[0]:
            raise Exception('y_amplitude %d out of range(-0.02~0.02)' % y_amplitude)
        if abs(rotation_angle) > self.rotation_angle_range[1] or abs(rotation_angle) < self.rotation_angle_range[0]:
            raise Exception('rotation_angle %d out of range(-10~10)' % rotation_angle)
        if body_height > self.body_height_range[1] or body_height < self.body_height_range[0]:
            raise Exception('body_height %d out of range(0.015~0.06)' % body_height)
        if step_height > self.step_height_range[1] or step_height < self.step_height_range[0]:
            raise Exception('step_height %d out of range(0.01~0.04)' % step_height)
        if arm_swap > self.arm_swap_range[1] or arm_swap < self.arm_swap_range[0]:
            raise Exception('arm_swap %d out of range(0~60)' % arm_swap)

        self.walking_param.period_time = step_velocity[0]
        self.walking_param.dsp_ratio = step_velocity[1]
        self.walking_param.y_swap_amplitude = step_velocity[2]
        self.walking_param.x_move_amplitude = x_amplitude
        self.walking_param.y_move_amplitude = y_amplitude
        self.walking_param.angle_move_amplitude = rotation_angle
        self.walking_param.init_z_offset = body_height
        self.walking_param.z_move_amplitude = step_height
        self.walking_param.pelvis_offset = pelvis_offset
        self.walking_param.hip_pitch_offset = hip_pitch_offset
        self.walking_param.z_swap_amplitude = z_swap_amplitude
        self.walking_param.arm_swing_gain = math.radians(arm_swap)
        self.param_pub.publish(self.walking_param)

    def set_step(self, step_velocity, x_amplitude, y_amplitude, rotation_angle, body_height=0.015, step_height=0.025,
                 pelvis_offset=3, hip_pitch_offset=20, z_swap_amplitude=0.01, arm_swap=30):
        '''
        以设置参数行走
        param step_velocity: 列表形式包含三个参数[period_time, dsp_ratio, y_swap_amplitude], 即周期(ms), 占地时间比例， 左右摆动幅度(m)
        param x_amplitude: x方向步幅(m)
        param y_amplitude: y方向步幅(m)
        param rotation_angle: 旋转幅度(deg)
        param body_height: 躯体离完全站直的距离(m)， 默认0.015
        param step_height: 抬腿高度(m)， 默认0.025
        param pelvis_offset: 髋关节左右摆动角度(deg)， 默认3
        param hip_pitch_offset: 髋关节前后倾斜角度(deg)， 默认20
        param z_swap_amplitude: 躯体上下摆动幅度(m)， 默认0.01
        param arm_swap: 手臂摆动幅度(deg)， 默认30
        '''
        try:
            self.update_param(step_velocity, x_amplitude, y_amplitude, rotation_angle, body_height, step_height,
                 pelvis_offset, hip_pitch_offset, z_swap_amplitude, arm_swap)
            if not self.have_start:
                self.have_start = True
                res = rospy.ServiceProxy('/walking/get_param', GetWalkingParam)()
                self.walking_param = res.parameters
                rospy.ServiceProxy('/walking/command', SetWalkingCommand)('start')
        except BaseException as e:
            print(e)
            return

    def move(self, step_velocity, x_amplitude, y_amplitude, rotation_angle):
        '''
        param step_velocity: 速度选择分三档分别为 1，2，3, 4速度由快到慢
        param x_amplitude: x方向步幅(m)
        param y_amplitude: y方向步幅(m)
        param rotation_angle: 旋转幅度(deg)
        '''
        if 0 < step_velocity < 5:
            self.set_step(self.dsp_ratio[step_velocity - 1], x_amplitude, y_amplitude, rotation_angle, body_height=0.015, step_height=0.025, arm_swap=30)

    def stop(self):
        rospy.ServiceProxy('/walking/command', SetWalkingCommand)('stop')
        self.have_start = False

    def disable(self):
        rospy.ServiceProxy('/walking/command', SetWalkingCommand)('disable')
        self.have_start = False

    def enable(self):
        rospy.ServiceProxy('/walking/command', SetWalkingCommand)('enable')

if __name__ == '__main__':
    rospy.init_node('walk_test')
    gait_manager = GaitManager()
    gait_manager.move(1, 0.02, 0, 0)
    rospy.sleep(2)
    gait_manager.stop()
    gait_manager.move(2, -0.02, 0, 0)
    rospy.sleep(2)
    gait_manager.stop()
    gait_manager.move(3, 0, 0, 10)
    rospy.sleep(2)
    gait_manager.stop()
