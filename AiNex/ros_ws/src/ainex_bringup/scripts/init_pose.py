#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2023/05/22
# 开机后的初始姿态
import rospy
from hiwonder_servo_msgs.msg import CommandDuration

class InitPose:
    def __init__(self, name):
        rospy.init_node(name)

        namespace = rospy.get_namespace()
        param = rospy.get_param('~')
        
        joint_position_pub = {}
        for joint_name in param:
            joint_position_pub[joint_name] = rospy.Publisher(namespace + joint_name + "_controller/command_duration", CommandDuration, queue_size=1)
        while not rospy.is_shutdown():
            try:
                if rospy.get_param(namespace + 'hiwonder_servo_manager/init_finish') and rospy.get_param(namespace + 'joint_states_publisher/init_finish'):
                    break
            except:
                rospy.sleep(0.1)
        rospy.sleep(0.3) 
        for joint_name in param:
            joint_position_pub[joint_name].publish(CommandDuration(data=param[joint_name], duration=2000))
            rospy.sleep(0.02)
        rospy.sleep(2.5)
        rospy.set_param('~init_finish', True)
        rospy.loginfo('init pose finish')
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rospy.set_param('~init_finish', True)
            rate.sleep()

if __name__ == '__main__':
    InitPose('init_pose')
