#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/06/20
# @author:aiden
import os
import copy
import math
import rospy
import numpy as np
from ainex_sdk import common
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from ainex_interfaces.msg import WalkingParam
from ainex_kinematics.kinematics import LegIK
from ainex_kinematics.walking_module import WalkingModule
from ainex_interfaces.srv import SetWalkingParam, GetWalkingParam, SetWalkingCommand

class JointPosition:
    def __init__(self):
        self.present_position = 0.0
        self.goal_position = 0.0

class Controller:
    def __init__(self, name):
        rospy.init_node(name)
        rospy.set_param('/init_pose/init_finish', False)
        self.init_pose_loaded = False
        self.walking_enable = True
        self.enable = False
        self.stop = False

        self.ik = LegIK() 
        leg_data = self.ik.get_leg_length()
        self.leg_length = leg_data[0] + leg_data[1] + leg_data[2]
        self.joint_position_pub = {}
        self.present_joint_state = {}

        self.joint_index = {'r_hip_yaw':   0,
                            'r_hip_roll':  1,
                            'r_hip_pitch': 2,
                            'r_knee':      3,
                            'r_ank_pitch': 4,
                            'r_ank_roll':  5,
                            'l_hip_yaw':   6,
                            'l_hip_roll':  7,
                            'l_hip_pitch': 8,
                            'l_knee':      9,
                            'l_ank_pitch': 10,
                            'l_ank_roll':  11,
                            'r_sho_pitch': 12,
                            'l_sho_pitch': 13}

        self.joint_id = {'r_hip_yaw': 12,
                         'r_hip_roll':  10,
                         'r_hip_pitch': 8,
                         'r_knee':      6,
                         'r_ank_pitch': 4,
                         'r_ank_roll':  2,
                         'l_hip_yaw':   11,
                         'l_hip_roll':  9,
                         'l_hip_pitch': 7,
                         'l_knee':      5,
                         'l_ank_pitch': 3,
                         'l_ank_roll':  1,
                         'r_sho_pitch': 14,
                         'l_sho_pitch': 13}
        for joint_name in self.joint_index:
            self.present_joint_state[joint_name] = JointPosition()
            self.joint_position_pub[joint_name] = rospy.Publisher("/" + joint_name + "_controller/command", Float64, queue_size=1)

        self.walking_param = common.get_yaml_data('/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/walking_param.yaml')

        self.init_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              math.radians(0.0), math.radians(-0.0)]
        self.walking_module = WalkingModule(self.ik, self.walking_param, self.joint_index, self.init_position, self.walking_param['trajectory_step_s'])
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/init_pose/init_finish'):
                    break
            except:
                rospy.sleep(0.1)
        
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback, queue_size=1)
        rospy.Service('/walking/command', SetWalkingCommand, self.walking_command_callback)
        rospy.Subscriber('/walking/set_param', WalkingParam, self.set_walking_param_callback, queue_size=1)
        rospy.Service('/walking/get_param', GetWalkingParam, self.get_walking_param_callback)
        
        self.last_position = None
        
        self.servo_max_move = 0
        self.speed = 0.2/math.radians(60)  # 舵机速度0.2s/60deg
        rospy.sleep(1)
        
        rospy.loginfo('ainex controller init finish')
        while not rospy.is_shutdown():
            if self.walking_enable:
                if self.init_pose_loaded:
                    t1 = rospy.get_time()
                    result = self.walking_module.run(self.present_joint_state)
                    if result[0] >= 0.24:
                        self.stop = False
                        
                        t2 = rospy.get_time()
                        cal_time = t2 - t1
                        if cal_time < self.servo_max_move*self.speed:
                            rospy.sleep(self.servo_max_move*self.speed - cal_time)
                        else:
                            if cal_time < 0.002:
                                rospy.sleep(0.002 - cal_time)
                        self.servo_max_move = 0
                        for joint_name in self.present_joint_state:
                            if self.walking_param['arm_swing_gain'] != 0 or (self.walking_param['arm_swing_gain'] == 0 and joint_name != 'r_sho_pitch' and joint_name != 'l_sho_pitch'):
                                self.joint_position_pub[joint_name].publish(result[1][joint_name].goal_position)
                                if self.last_position is not None:
                                    d = abs(self.last_position[1][joint_name].goal_position - result[1][joint_name].goal_position)
                                    if self.servo_max_move < d:
                                        self.servo_max_move = d
                        self.last_position = copy.deepcopy(result)
                    elif self.walking_module.walk_finish:
                        rospy.sleep(0.001)
                        self.stop = True
                    else:
                        rospy.sleep(0.001)
                else:
                    rospy.sleep(0.001)
            else:
                rospy.sleep(0.001)

    def joint_states_callback(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i] in self.present_joint_state:
                self.present_joint_state[msg.name[i]].present_position = msg.position[i]
        if not self.init_pose_loaded:
            for joint_name in self.present_joint_state:
                self.present_joint_state[joint_name].goal_position = self.present_joint_state[joint_name].present_position
            self.init_pose_loaded = True

    def walking_command_callback(self, msg):
        if msg.command == "start":
            self.walking_enable = True
            self.walking_module.start()
            rospy.loginfo('walking start')
        elif msg.command == "stop":
            self.walking_module.stop()
            while not self.stop:
                rospy.sleep(0.01)
            rospy.loginfo('walking stop')
        elif msg.command == 'enable':
            self.walking_enable = True
            rospy.loginfo('walking enable')
        elif msg.command == 'disable':
            self.walking_module.stop()
            while not self.stop:
                rospy.sleep(0.01)
            self.walking_enable = False
            rospy.loginfo('walking disable')

        return True

    def set_walking_param_callback(self, msg):
        self.walking_param['init_x_offset'] = msg.init_x_offset
        self.walking_param['init_y_offset'] = msg.init_y_offset
        self.walking_param['init_z_offset'] = msg.init_z_offset
        self.walking_param['init_roll_offset'] = msg.init_roll_offset
        self.walking_param['init_pitch_offset'] = msg.init_pitch_offset
        self.walking_param['init_yaw_offset'] = msg.init_yaw_offset
        self.walking_param['hip_pitch_offset'] = msg.hip_pitch_offset
        self.walking_param['period_time'] = msg.period_time
        self.walking_param['dsp_ratio'] = msg.dsp_ratio
        self.walking_param['step_fb_ratio'] = msg.step_fb_ratio
        self.walking_param['x_move_amplitude'] = msg.x_move_amplitude
        self.walking_param['y_move_amplitude'] = msg.y_move_amplitude
        self.walking_param['z_move_amplitude'] = msg.z_move_amplitude
        self.walking_param['angle_move_amplitude'] = msg.angle_move_amplitude
        self.walking_param['y_swap_amplitude'] = msg.y_swap_amplitude
        self.walking_param['z_swap_amplitude'] = msg.z_swap_amplitude
        self.walking_param['pelvis_offset'] = msg.pelvis_offset
        self.walking_param['move_aim_on'] = msg.move_aim_on
        self.walking_param['arm_swing_gain'] = msg.arm_swing_gain

        self.walking_module.set_walking_param(self.walking_param)

    def get_walking_param_callback(self, msg):
        self.walking_param = self.walking_module.get_walking_param()
        
        param = WalkingParam()
        param.init_x_offset = self.walking_param['init_x_offset']
        param.init_y_offset = self.walking_param['init_y_offset']
        param.init_z_offset = self.walking_param['init_z_offset']
        param.init_roll_offset = self.walking_param['init_roll_offset']
        param.init_pitch_offset = self.walking_param['init_pitch_offset']
        param.init_yaw_offset = self.walking_param['init_yaw_offset']
        param.hip_pitch_offset = self.walking_param['hip_pitch_offset']
        param.period_time = self.walking_param['period_time']
        param.dsp_ratio = self.walking_param['dsp_ratio']
        param.step_fb_ratio = self.walking_param['step_fb_ratio']
        param.x_move_amplitude = self.walking_param['x_move_amplitude']
        param.y_move_amplitude = self.walking_param['y_move_amplitude']
        param.z_move_amplitude = self.walking_param['z_move_amplitude']
        param.angle_move_amplitude = self.walking_param['angle_move_amplitude']
        param.y_swap_amplitude = self.walking_param['y_swap_amplitude']
        param.z_swap_amplitude = self.walking_param['z_swap_amplitude']
        param.pelvis_offset = self.walking_param['pelvis_offset']
        param.move_aim_on = self.walking_param['move_aim_on']
        param.arm_swing_gain = self.walking_param['arm_swing_gain']
        
        return param 

if __name__ == '__main__':
    Controller('ainex_controller')
