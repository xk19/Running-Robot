#!/usr/bin/python3
# coding=utf8
import rospy
from std_srvs.srv import SetBool
from ainex_interfaces.srv import SetFloat

rospy.init_node('buzzer_node')
# 以10hz的频率响一次
rospy.ServiceProxy('/sensor/buzzer/set_buzzer_frequency', SetFloat)(10)
rospy.sleep(1)
# 打开蜂鸣器
rospy.ServiceProxy('/sensor//buzzer/set_buzzer_state', SetBool)(True)
# 延时0.5s关闭
rospy.sleep(0.5)
rospy.ServiceProxy('/sensor/buzzer/set_buzzer_state', SetBool)(False)
