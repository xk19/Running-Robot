#!/usr/bin/python3
# coding=utf8
import rospy
from std_msgs.msg import Int32

def button_callback(msg):
    rospy.loginfo('button_state: %s'%msg.data)

rospy.init_node('button_node')
# 订阅按钮状态
rospy.Subscriber('/sensor/button/get_button_state', Int32, button_callback)
try:
    rospy.spin()
except keyboardinterrupt:
    print("shutting down")
