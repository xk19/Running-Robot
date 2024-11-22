#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/04
# @author:aiden
# 跌倒起立
import cv2
import math
import rospy
import signal
import numpy as np
from ainex_sdk import common
from threading import RLock
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty, EmptyResponse
from ainex_interfaces.srv import SetFloat
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from ainex_kinematics.gait_manager import GaitManager
from hiwonder_servo_controllers.motion_manager import MotionManager

class FallRiseNode:
    lie_to_stand_action_name = 'lie_to_stand'
    recline_to_stand_action_name = 'recline_to_stand'
    
    def __init__(self, name):
        self.name = name
        rospy.init_node(self.name)
        self.debug = False
        self.running = True
        self.point1 = [240, 230]
        self.point2 = [240, 30]
        self.count_lie = 0
        self.count_recline = 0
        self.state = 'stand'
        self.start = False
        self.image = np.zeros((300, 480), np.uint8)
        self.imu_sub = None

        self.lock = RLock()
        self.gait_manager = GaitManager()
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.motion_manager = MotionManager(self.joints_pub, '/home/ubuntu/software/ainex_controller/ActionGroups')
        signal.signal(signal.SIGINT, self.shutdown)

        rospy.Service('~start', Empty, self.start_srv_callback)  # 开始玩法
        rospy.Service('~stop', Empty, self.stop_srv_callback)  # 停止玩法

        # 等待舵机准备就绪
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/hiwonder_servo_manager/init_finish') and rospy.get_param('/joint_states_publisher/init_finish'):
                    break
            except:
                rospy.sleep(0.1)
        rospy.sleep(0.2)
        rospy.set_param('~init_finish', True)
        common.loginfo('%s init_finish' % self.name) 

        if rospy.get_param('~start', True):
            self.start_srv_callback(None)
            common.loginfo('start fall_rise')

    def shutdown(self, signum, frame):
        self.running = False

        common.loginfo('%s shutdown' % self.name)

    def start_srv_callback(self, msg):
        with self.lock:
            if self.imu_sub is None:
                self.imu_sub = rospy.Subscriber('/sensor/imu/imu_raw', Imu, self.imu_callback)
            self.start = True
        common.loginfo('%s start' % self.name)
        return EmptyResponse()

    def stop_srv_callback(self, msg):
        # 停止巡线
        with self.lock:
            self.start = False
            if self.imu_sub is not None:
                self.imu_sub.unregister()
                self.imu_sub = None
        common.loginfo('%s stop' % self.name)
        return EmptyResponse()

    def rotate(self, ps, m):
        pts = np.float32(ps).reshape([-1, 2])  # 要映射的点
        pts = np.hstack([pts, np.ones([len(pts), 1])]).T
        target_point = np.dot(m, pts).astype(np.int_)
        target_point = [[target_point[0][x], target_point[1][x]] for x in range(len(target_point[0]))]
        
        return target_point

    def rotate_point(self, center_point, corners, angle):
        '''
        获取一组点绕一点旋转后的位置
        :param center_point:
        :param corners:
        :param angle:
        :return:
        '''
        # points [[x1, y1], [x2, y2]...]
        # 角度
        M = cv2.getRotationMatrix2D((center_point[0], center_point[1]), angle, 1)
        out_points = self.rotate(corners, M)

        return out_points

    def imu_callback(self, msg):
        angle = abs(int(math.degrees(math.atan2(msg.linear_acceleration.y, msg.linear_acceleration.z)))) #转化为角度值
        if self.state == 'stand':
            if angle < 30:
                self.count_lie += 1
            else:
                self.count_lie = 0
            if angle > 150:
                self.count_recline += 1
            else:
                self.count_recline = 0
            if self.count_lie > 100:
                self.count_lie = 0
                self.state = 'lie_to_stand'
            elif self.count_recline > 100:
                self.count_recline = 0
                self.state = 'recline_to_stand'
            rospy.sleep(0.01)
        if self.debug:
            image = self.image.copy()
            # 通过图像显示方向
            point = self.rotate_point(self.point1, [self.point2], 90 - angle)[0]
            cv2.line(image, tuple(self.point1), tuple(point), (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(image, str(angle), (225, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow('image', image)
            key = cv2.waitKey(1)
 
    def run(self):
        while self.running:
            if self.start:
                if self.state != 'stand':
                    rospy.ServiceProxy('/sensor/buzzer/set_buzzer_frequency', SetFloat)(10)
                    rospy.sleep(1)
                    self.gait_manager.disable()
                    if self.state == 'lie_to_stand':
                        common.loginfo('lie_to_stand')
                        self.motion_manager.runAction(self.lie_to_stand_action_name)
                    elif self.state == 'recline_to_stand':
                        common.loginfo('recline_to_stand')
                        self.motion_manager.runAction(self.recline_to_stand_action_name)
                    rospy.sleep(0.5)
                    self.state = 'stand'
                else:
                    rospy.sleep(0.01)
            else:
                rospy.sleep(0.01)

        rospy.signal_shutdown('shutdown')

if __name__ == '__main__':
    FallRiseNode('fall_rise').run()
