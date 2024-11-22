#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/09
# @author:aiden
import time
import copy
import math
import rospy
from ainex_sdk import common
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String, Bool
from ainex_interfaces.msg import WalkingParam
from ainex_kinematics.kinematics import LegIK
from ainex_kinematics.walking_module import WalkingModule
from ainex_kinematics.motion_manager import MotionManager
from ainex_interfaces.srv import SetWalkingParam, GetWalkingParam, SetWalkingCommand, GetWalkingState

class JointPosition:
    def __init__(self):
        self.present_position = 0.0
        self.goal_position = 0.0

class Controller:
    RADIANS_PER_ENCODER_TICK = 240 * 3.1415926 / 180 / 1000
    ENCODER_TICKS_PER_RADIAN = 180 / 3.1415926 / 240 * 1000
    joint_index = {'r_hip_yaw':   0,
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
    
    # 舵机关节名称和舵机id的对应关系
    joint_id = {'r_hip_yaw':   12,
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
                'l_sho_pitch': 13,
                'l_sho_roll':  15,
                'r_sho_roll':  16,
                'l_el_pitch':  17,
                'r_el_pitch':  18,
                'l_el_yaw':    19,
                'r_el_yaw':    20,
                'l_gripper':   21,
                'r_gripper':   22,
                'head_pan':    23,
                'head_tilt':   24}

    def __init__(self, name):
        rospy.init_node(name)
        rospy.set_param('/init_pose/init_finish', False)
        self.gazebo_sim = rospy.get_param('gazebo_sim', False)
        ####################舵机脉宽和弧度的关系#############
        self.joint_angles_convert_coef = {}
        items_ = rospy.get_param('~controllers').items()
        for ctl_name, ctl_params in items_:
            if ctl_params['type'] == 'JointPositionController':
                initial_position_raw = ctl_params['servo']['init']
                min_angle_raw = ctl_params['servo']['min']
                max_angle_raw = ctl_params['servo']['max']
                flipped = min_angle_raw > max_angle_raw

                if flipped:
                    self.joint_angles_convert_coef[ctl_params['servo']['id']] = [initial_position_raw, -self.ENCODER_TICKS_PER_RADIAN]
                else:
                    self.joint_angles_convert_coef[ctl_params['servo']['id']] = [initial_position_raw, self.ENCODER_TICKS_PER_RADIAN]
        #########初始姿态#########
        init_pose_data = rospy.get_param('~init_pose')
        self.motion_manager = MotionManager()
        data = []
        for joint_name in init_pose_data:
            id_ = self.joint_id[joint_name]
            angle = init_pose_data[joint_name]
            pulse = self.angle2pulse(id_, angle)
            data.extend([[id_, pulse]])
        self.motion_manager.set_servos_position(1000, data)
        #########################

        self.walking_enable = True
        self.walk_finish = False
        self.count_step = 0
        self.stop = False

        self.ik = LegIK() 
        leg_data = self.ik.get_leg_length()
        self.leg_length = leg_data[0] + leg_data[1] + leg_data[2]
        self.joint_position_pub = {}
        self.present_joint_state = {}

        for joint_name in self.joint_index:
            self.present_joint_state[joint_name] = JointPosition()
            if self.gazebo_sim:
                self.joint_position_pub[joint_name] = rospy.Publisher("/" + joint_name + "_controller/command", Float64, queue_size=1)
            else:
                self.present_joint_state[joint_name].present_position = init_pose_data[joint_name]
        
        self.walking_param = common.get_yaml_data('/home/ubuntu/ros_ws/src/ainex_driver/ainex_kinematics/config/walking_param.yaml')

        self.init_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              math.radians(0.0), math.radians(-0.0)]
        self.walking_module = WalkingModule(self.ik, self.walking_param, self.joint_index, self.init_position, self.walking_param['trajectory_step_s'])

        if self.gazebo_sim:
            rospy.Subscriber('/joint_states', JointState, self.joint_states_callback, queue_size=1)
        rospy.Service('/walking/command', SetWalkingCommand, self.walking_command_callback)
        rospy.Subscriber('/walking/set_param', WalkingParam, self.set_walking_param_callback, queue_size=1)
        rospy.Service('/walking/get_param', GetWalkingParam, self.get_walking_param_callback)
        rospy.Service('/walking/is_walking', GetWalkingState, self.is_walking_callback)
        self.walk_state_pub = rospy.Publisher('/walking/is_walking', Bool, queue_size=1)
        
        self.last_position = None
        
        self.stop_stamp = self.walking_param['trajectory_step_s'] 
        self.servo_control_cycle = self.walking_param['servo_control_cycle']
        self.servo_max_move = 0
        self.speed = 0.2/math.radians(60)  # 舵机最大速度0.2s/60deg
        rospy.sleep(1)
        rospy.loginfo('ainex controller init finish')

    def run(self): 
        next_time = time.monotonic()
        while not rospy.is_shutdown():
            if self.walking_enable:
                joint_max_move, times_stamp, joint_state = self.walking_module.run(self.present_joint_state)
                if joint_max_move >= 0.24:
                    if self.stop:
                        self.walk_state_pub.publish(True)
                    self.stop = False
                    
                    curr_time = time.monotonic()
                    delta_sec = curr_time - next_time
                    delay_time = self.servo_max_move*self.speed - delta_sec
                    if delay_time > 0:
                        if delta_sec + delay_time < self.servo_control_cycle:
                            # print(1, 0.003 - delta_sec)
                            rospy.sleep(self.servo_control_cycle - delta_sec)
                        else:
                            # print(2, delay_time)
                            rospy.sleep(delay_time)
                    else:
                        if delta_sec < self.servo_control_cycle:
                            # print(3, 0.008 - delta_sec)
                            rospy.sleep(self.servo_control_cycle - delta_sec)
                    self.servo_max_move = 0
                    data = []
                    for joint_name in self.present_joint_state:
                        if self.walking_param['arm_swing_gain'] != 0 or (self.walking_param['arm_swing_gain'] == 0 and joint_name != 'r_sho_pitch' and joint_name != 'l_sho_pitch'):
                            goal_position = joint_state[joint_name].goal_position
                            if not self.gazebo_sim:
                                id_ = self.joint_id[joint_name]
                                pulse = self.angle2pulse(id_, goal_position)
                                data.extend([[id_, pulse]])
                                self.present_joint_state[joint_name].present_position = goal_position
                            else:
                                self.joint_position_pub[joint_name].publish(goal_position)
                            if self.last_position is not None:
                                d = abs(self.last_position[joint_name].goal_position - goal_position)
                                if self.servo_max_move < d:
                                    self.servo_max_move = d
                    self.motion_manager.set_servos_position(int(self.servo_control_cycle*1000), data)
                    next_time = time.monotonic()
                    self.last_position = copy.deepcopy(joint_state)
                elif self.walking_module.walk_finish:
                    if not self.stop:
                        self.walk_state_pub.publish(False)
                    self.servo_max_move = 0
                    rospy.sleep(0.001)
                    self.stop = True
                else:
                    self.servo_max_move = 0
                    rospy.sleep(0.001)
                if times_stamp >= self.walking_param['period_time']/1000.0 - self.walking_param['trajectory_step_s']:
                    if self.walking_param['period_times'] != 0:
                        self.count_step += 1
                        if self.walking_param['period_times'] == self.count_step:
                            self.count_step = 0
                            self.walking_param['period_times'] = 0
                            self.walking_module.stop()
                    else:
                        self.count_step = 0
            else:
                rospy.sleep(0.001)

    def angle2pulse(self, id_, angle):
        return self.joint_angles_convert_coef[id_][0] + int(round(angle * self.joint_angles_convert_coef[id_][1]))

    def joint_states_callback(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i] in self.present_joint_state:
                self.present_joint_state[msg.name[i]].present_position = msg.position[i]

    def is_walking_callback(self, msg):
        return [self.stop, "is_walking"]

    def walking_command_callback(self, msg):
        if msg.command == "start":
            self.walking_enable = True
            self.walking_module.start()
            self.walking_finish = False
        elif msg.command == "stop":
            self.walking_module.stop()
            while not self.stop:
                rospy.sleep(0.01)
        elif msg.command == 'enable':
            self.walking_enable = True
        elif msg.command == 'disable':
            self.walking_module.stop()
            while not self.stop:
                rospy.sleep(0.01)
            self.walking_enable = False

        return True

    def set_walking_param_callback(self, msg):
        self.walking_param['period_times'] = msg.period_times
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
        param.period_times = 0
        
        return param 

if __name__ == '__main__':
    Controller('ainex_controller').run()
