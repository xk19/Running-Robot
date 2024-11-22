#!/usr/bin/env python3
# encoding: utf-8
# Date:2023/06/04
import rospy
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from ainex_kinematics.gait_manager import GaitManager
from hiwonder_servo_controllers.motion_manager import MotionManager

rospy.init('gait_demo')
# 调用上位机生成的动作
joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
motion_manager = MotionManager(joints_pub, '/home/ubuntu/software/ainex_controller/ActionGroups')
gait_manager = GaitManager()
rospy.sleep(0.2)

# 控制机器人行走
gait_manager.move(1, 0.02, 0, 0)
rospy.sleep(3)  # 以速度1前进3秒然后停下
gait_manager.stop()

# 如果需要切换到动作组需要先关掉
gait_manager.disable()
motion_manager.runAction('left_shot')

gait_manager.move(2, 0, 0, 5)
rospy.sleep(3)  # 右转3秒

gait_manager.move(3, 0.02, 0, 5)
rospy.sleep(3)  # 前进同时右转3秒

gait_manager.move(2, 0, 0.02, 0)
rospy.sleep(3)  # 右移3秒

gait_manager.move(2, -0.02, 0, 0)
rospy.sleep(3)  # 后退3秒然后停下
gait_manager.stop()
