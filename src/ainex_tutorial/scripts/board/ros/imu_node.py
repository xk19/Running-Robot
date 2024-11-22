#!/usr/bin/python3
# coding=utf8
import rospy
from sensor_msgs.msg import MagneticField, Imu

def imu_callback(msg):
    rospy.loginfo(msg)
    rospy.loginfo('\n')

def imu_mag_callback(msg):
    rospy.loginfo(msg)
    rospy.loginfo('\n')

rospy.init_node('imu_node')
# 订阅imu数据
rospy.Subscriber('/sensor/imu/imu_raw', Imu, imu_callback)
rospy.Subscriber('/sensor/imu/imu_mag', MagneticField, imu_mag_callback)
try:
    rospy.spin()
except keyboardinterrupt:
    print("shutting down")
