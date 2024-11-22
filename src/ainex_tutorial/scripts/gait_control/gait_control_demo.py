#!/usr/bin/env python3
# encoding: utf-8
# Date:2023/07/10
import rospy
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager

rospy.init_node('gait_control_demo')
# 调用上位机生成的动作, 参数为动作组存储的路径
motion_manager = MotionManager(action_path='/home/ubuntu/software/ainex_controller/ActionGroups')
# 步态控制库
gait_manager = GaitManager()
rospy.sleep(0.2)

# 控制机器人行走
gait_manager.move(1, 0.02, 0, 0)
rospy.sleep(3)  # 以速度1前进3秒然后停下
gait_manager.stop()

# 如果需要切换到动作组需要先关掉步态控制
gait_manager.disable()
motion_manager.runAction('left_shot')

motion_manager.runAction('hand_open')
gait_manager.move(2, -0.02, 0, 0, arm_swap=0)  # 要保持手臂动作，需要关闭摆臂动作
rospy.sleep(3)  # 后退3秒然后停下
gait_manager.stop()

gait_manager.move(2, 0, 0, 5)
rospy.sleep(3)  # 右转3秒

gait_manager.move(3, 0.02, 0, 5)
rospy.sleep(3)  # 前进同时右转3秒

gait_manager.move(2, 0, 0.02, 0, step_num=3)  # 控制行走步数

gait_manager.move(2, -0.02, 0, 0)
rospy.sleep(3)  # 后退3秒然后停下
gait_manager.stop()

# 更多参数调节的控制，各参数含义请参考ainex_kinematics/src/ainex_kinematics/gait_manager
gait_param = gait_manager.get_gait_param()  # 获取当前步态参数
gait_param['pelvis_offset'] = 5
gait_param['step_height'] = 0.02
gait_param['z_swap_amplitude'] = 0.006
dsp = [400, 0.2, 0.02]
gait_manager.set_step(dsp, 0.02, 0, 0, gait_param, arm_swap=30, step_num=0)
rospy.sleep(3)
gait_manager.stop()
