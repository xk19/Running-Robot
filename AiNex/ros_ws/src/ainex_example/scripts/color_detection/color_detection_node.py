#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/02/04
# @author:aiden
# 订阅摄像头图像，进行图像处理，发布相应的物体姿态信息和图像
import cv2
import rospy
import signal
import numpy as np
from threading import RLock
from sensor_msgs.msg import Image
from std_srvs.srv import Empty, EmptyResponse
from ainex_interfaces.msg import ObjectInfo, ObjectsInfo, ColorsDetect

from ainex_sdk import common
from color_detection import ColorDetection

LAB_CONFIG_PATH = '/home/ubuntu/software/lab_tool/lab_config.yaml'
class ColorDetectionNode:
    def __init__(self, name):
        # 初始化节点
        rospy.init_node(name, log_level=rospy.INFO)
        self.name = name

        self.image = None
        self.running = True
        self.image_sub = None
        self.start_detect = False
        
        self.lock = RLock()
        signal.signal(signal.SIGINT, self.shutdown)

        # 获取参数
        lab_config = common.get_yaml_data(LAB_CONFIG_PATH)
        
        self.debug = rospy.get_param('~debug', False)
        self.camera = rospy.get_param('/camera')
        
        # 实例化颜色识别类
        self.detect = ColorDetection(lab_config['lab']['Mono'], ColorsDetect(), [160, 120])

        # 检测图像发布
        self.image_pub = rospy.Publisher('~image_result', Image, queue_size=1)

        # 物体位姿发布
        self.color_info_pub = rospy.Publisher('/object/pixel_coords', ObjectsInfo, queue_size=1)

        rospy.Subscriber('~update_detect', ColorsDetect, self.update_detect, queue_size=1)
        
        rospy.Service('~enter', Empty, self.enter_func)
        rospy.Service('~exit', Empty, self.exit_func)
        rospy.Service('~start', Empty, self.start_func)
        rospy.Service('~stop', Empty, self.stop_func)
        rospy.Service('~update_lab', Empty, self.update_lab)

        self.enable_display = rospy.get_param('~enable_display', False)
        rospy.sleep(0.2)
        common.loginfo("%s init finish"%self.name)
        
        if self.debug:
            self.enter_func(None)
            self.start_func(None)

        self.image_proc()

    def shutdown(self, signum, frame):
        with self.lock:
            self.running = False
            
            common.loginfo('%s shutdown'%self.name)
    
    # 开启订阅
    def enter_func(self, msg):
        # 获取参数
        with self.lock:
            self.image = None
            if self.image_sub is None:
                self.image_sub = rospy.Subscriber('/%s/image_raw'%self.camera['camera_name'], Image, self.image_callback)  # 订阅图像
        
        common.loginfo("%s enter"%self.name)
        
        return EmptyResponse()

    # 注销订阅
    def exit_func(self, msg):
        with self.lock:
            self.stop_func(Empty())
            if self.image_sub is not None:
                self.image_sub.unregister()
                self.image_sub = None
            self.image = None
        
        common.loginfo('%s exit'%self.name)

        return EmptyResponse()

    # 开启检测
    def start_func(self, msg):
        with self.lock:
            self.start_detect = True
        
        common.loginfo("%s start"%self.name)

        return EmptyResponse()

    # 停止检测
    def stop_func(self, msg):
        with self.lock:
            self.start_detect = False
        
        common.loginfo("%s stop"%self.name)

        return EmptyResponse()

    # 更新颜色列表
    def update_detect(self, msg):
        self.detect.update_detect_info(msg.data)
        
        common.loginfo('update detect')

    # 更新config参数
    def update_lab(self, msg):
        config = common.get_yaml_data(LAB_CONFIG_PATH)
        self.detect.update_lab_config(config)
        
        common.loginfo('%s update lab'%self.name)

        return EmptyResponse()

    def image_proc(self):
        while self.running:
            with self.lock:
                if self.image is not None:
                    time_start = rospy.get_time()
                    if self.start_detect:  # 如果开启检测
                        frame_result, poses = self.detect.detect(self.image)  # 颜色检测
                        colors_info = []
                        for p in poses:
                            color_info = ObjectInfo()
                            color_info.label = p[0]
                            color_info.x = p[1][0]
                            color_info.y = p[1][1]
                            color_info.width = p[2][0]
                            color_info.height = p[2][1]
                            color_info.radius = p[3]
                            color_info.angle = p[4]
                            colors_info.append(color_info)
                        self.color_info_pub.publish(colors_info)  # 发布位姿
                        time_d = 0.03 - (rospy.get_time() - time_start)
                        if time_d > 0:
                            rospy.sleep(time_d)
                    else:
                        frame_result = self.image
                        rospy.sleep(0.03)
                    ros_image = common.cv2_image2ros(frame_result, self.name)  # opencv格式转为ros
                    self.image_pub.publish(ros_image)  # 发布图像
                    if self.enable_display:
                        cv2.imshow('color_detection', frame_result)
                        key = cv2.waitKey(1)
                        if key != -1:
                            self.running = False
                else:
                    rospy.sleep(0.01)

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                           buffer=ros_image.data)  # 将ros格式图像消息转化为opencv格式
        self.image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

if __name__ == '__main__':
    ColorDetectionNode('color_detection')
