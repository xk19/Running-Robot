#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/03
# @author:aiden
# 视觉巡线节点
import rospy
import signal
from std_msgs.msg import String
from ainex_sdk import common
from ainex_example.common import Common
from ainex_example.visual_patrol import VisualPatrol
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect

import cv2
import math
import copy
import numpy as np
import time
from color_detection_yy import ColorDetection_yy

class VisualPatrolNode(Common):
    # 按顺序检测三个roi，如果检测到黑线立刻跳出
    roi = [ # [ROI, weight]
            (5/12, 6/12, 1/4, 3/4),
            (6/12, 7/12, 1/4, 3/4),
            (7/12, 8/12, 1/4, 3/4)
          ]
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True
        self.count_miss = 0
        self.object_info = None
        self.image_process_size = [160, 120]
        self.head_pan_init = 500   # 左右舵机的初始值
        self.head_tilt_init = 250  # 上下舵机的初始值
        super().__init__(name, self.head_pan_init, self.head_tilt_init)

        self.visual_patrol = VisualPatrol(self.gait_manager)
        signal.signal(signal.SIGINT, self.shutdown)

        # 订阅颜色识别结果
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色

        if rospy.get_param('~start', True):
            # 通知颜色识别准备，此时只显示摄像头原画
            target_color = rospy.get_param('~color', 'black')
            self.enter_func(None)
            self.set_color_srv_callback(String(target_color))  # 设置识别黑色
            self.start_srv_callback(None)  # 开启颜色识别
            common.loginfo('start track %s lane' % target_color)

    def shutdown(self, signum, frame):
        with self.lock:
            self.running = False
            common.loginfo('%s shutdown' % self.name)

    def set_color_srv_callback(self, msg):
        # 设置颜色
        param = ColorDetect()
        param.color_name = msg.data
        param.use_name = True
        param.detect_type = 'line'
        param.image_process_size = self.image_process_size
        param.line_roi.up.y_min = int(self.roi[0][0] * self.image_process_size[1])
        param.line_roi.up.y_max = int(self.roi[0][1] * self.image_process_size[1])
        param.line_roi.up.x_min = int(self.roi[0][2] * self.image_process_size[0])
        param.line_roi.up.x_max = int(self.roi[0][3] * self.image_process_size[0])

        param.line_roi.center.y_min = int(self.roi[1][0] * self.image_process_size[1])
        param.line_roi.center.y_max = int(self.roi[1][1] * self.image_process_size[1])
        param.line_roi.center.x_min = int(self.roi[1][2] * self.image_process_size[0])
        param.line_roi.center.x_max = int(self.roi[1][3] * self.image_process_size[0])

        param.line_roi.down.y_min = int(self.roi[2][0] * self.image_process_size[1])
        param.line_roi.down.y_max = int(self.roi[2][1] * self.image_process_size[1])
        param.line_roi.down.x_min = int(self.roi[2][2] * self.image_process_size[0])
        param.line_roi.down.x_max = int(self.roi[2][3] * self.image_process_size[0])

        param.min_area = 10
        param.max_area = self.image_process_size[0] * self.image_process_size[1]

        self.detect_pub.publish([param])

        common.loginfo('%s set_color' % self.name)

        return [True, 'set_color']

    def get_color_callback(self, msg):
        # 获取颜色识别结果
        if msg.data != []:
            self.count_miss = 0
            self.object_info = msg.data[0]
        elif self.start:
            self.count_miss += 1
            rospy.sleep(0.01)
            if self.count_miss > 100:
                self.count_miss = 0
                self.object_info = None

    def run(self):
        self.flag_run = 1
        while self.running:
            if self.object_info is not None and self.start:
                if self.flag_run:
                    self.flag_run = 0
                    self.b = "road"
                    self.set_color_srv_callback(String(self.b))
                self.visual_patrol.process(self.object_info.x, self.object_info.width)
                rospy.sleep(0.1)
            else:
                rospy.sleep(0.01)

        self.init_action()
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    VisualPatrolNode('visual_patrol').run()

