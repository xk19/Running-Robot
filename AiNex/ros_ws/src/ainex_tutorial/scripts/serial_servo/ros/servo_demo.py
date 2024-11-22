#!/usr/bin/env python3
# encoding: utf-8
# Date:2023/06/04
import rospy
from hiwonder_servo_msgs.srv import GetServoState 
from hiwonder_servo_msgs.msg import MultiRawIdPosDur, SetServoState
from hiwonder_servo_controllers.motion_manager import MotionManager

rospy.init_node('servo_demo')

joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
motion_manager = MotionManager(joints_pub, '/home/ubuntu/software/ainex_controller/ActionGroups')  # 动作组存放路径
set_servo_pub = rospy.Publisher('servo_control/set_servo_state', SetServoState, queue_size=1)
rospy.sleep(0.2)  # 延时等初始化完成

# 获取舵机当前电压
res = rospy.ServiceProxy('/servo_control/get_servo_state', GetServoState)('voltage', 23)
print('voltage: ' + str(round(res.value[0]/1000.0, 1)) + 'V')

# 当只连接一个舵机且不知道舵机的id时，可以使用此方法获取舵机id
# res = rospy.ServiceProxy('/servo_control/get_servo_state', GetServoState)('id', -1)
# print('id: ' + str(round(res.value[0])

# 可以通过这个方式来确认设定id的舵机是否连接
res = rospy.ServiceProxy('/servo_control/get_servo_state', GetServoState)('id', 23)
print('id: ', res.value)

# 获取指定舵机的偏差
res = rospy.ServiceProxy('/servo_control/get_servo_state', GetServoState)('deviation', 23)
print('deviation: ', res.value)

# 获取指定舵机的脉宽范围
res = rospy.ServiceProxy('/servo_control/get_servo_state', GetServoState)('pulse_range', 23)
print('pulse_range: ', res.value)

# 获取指定舵机的电压范围
res = rospy.ServiceProxy('/servo_control/get_servo_state', GetServoState)('voltage_range', 23)
print('voltage_range: ', res.value)

# 获取指定舵机的报警温度范围
res = rospy.ServiceProxy('/servo_control/get_servo_state', GetServoState)('temperature_range', 23)
print('temperature_range: ', res.value)

# 获取指定舵机当前温度
res = rospy.ServiceProxy('/servo_control/get_servo_state', GetServoState)('temperature', 23)
print('temperature: ', res.value)

# 获取指定舵机是否锁定
res = rospy.ServiceProxy('/servo_control/get_servo_state', GetServoState)('load_state', 23)
print('load_state: ', res.value)

# 将指定舵机ID设为新ID
# msg = SetServoState()
# msg.cmd = 'id'
# msg.param = [23, 25]
# res = set_servo_pub.publish(msg)

# 设定指定舵机偏差
# msg = SetServoState()
# msg.cmd = 'deviation'
# msg.param = [23, 10]
# res = set_servo_pub.publish(msg)

# 保存指定舵机偏差
# msg = SetServoState()
# msg.cmd = 'save_deviation'
# msg.param = [23]
# res = set_servo_pub.publish(msg)

# 设定指定舵机脉宽范围
# msg = setservostate()
# msg.cmd = 'pulse_range'
# msg.param = [23, 0, 1000]
# res = set_servo_pub.publish(msg)

# 设定指定舵机电压范围
# msg = setservostate()
# msg.cmd = 'voltage_range'
# msg.param = [23, 4500, 14000]
# res = set_servo_pub.publish(msg)

# 设定指定舵机报警温度范围
# msg = setservostate()
# msg.cmd = 'temperature_range'
# msg.param = [23, 85]
# res = set_servo_pub.publish(msg)

# 让指定舵机解除锁定
# msg = setservostate()
# msg.cmd = 'unload'
# msg.param = [23]
# res = set_servo_pub.publish(msg)

# 单个舵机运行
motion_manager.set_servos(500, ((23, 300), ))
rospy.sleep(0.5)  # 非阻塞需要根据运行时间加延时

# 多个舵机运行
motion_manager.set_servos(500, ((23, 500), (24, 500)))
rospy.sleep(0.5)

# 执行动作组
motion_manager.run_action('left_shot')  # 阻塞直到运行完，不用加延时
motion_manager.run_action('right_shot')
