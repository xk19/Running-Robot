#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/06/03
# @author:aiden
# 视觉巡线
import math
import rospy
import signal
from threading import RLock
from ainex_sdk import misc, common
from std_msgs.msg import Float64, String
from std_srvs.srv import Empty, EmptyResponse
from ainex_interfaces.srv import SetString
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from ainex_kinematics.gait_manager import GaitManager
from hiwonder_servo_controllers.motion_manager import MotionManager
from ainex_interfaces.msg import ObjectsInfo, ColorDetect, ColorsDetect

class VisualPatrol:
    # 按顺序检测三个roi，如果检测到黑线立刻跳出
    roi = [ # [ROI, weight]
            (50, 60, 40, 120), 
            (60, 70, 40, 120), 
            (70, 80, 40, 120)
          ]
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.center = None
        self.running = True
        self.start = False
        self.count_miss = 0
        self.time_stamp = 0
        self.head_time_stamp = 0
        self.head_position_state = None
       
        self.lock = RLock()
        signal.signal(signal.SIGINT, self.shutdown)
        
        # 机器人行走的库调用
        self.gait_manager = GaitManager()
        
        self.x_range = [0, 0.02]  # 前进脚步大小单位m
        self.yaw_range = [-10, 10]  # 转弯范围正负10度
        self.head_pan_init = 500  # 左右舵机的初始值
        self.head_tilt_init = 250 # 上下舵机的初始值
        
        # 更新颜色识别参数
        self.detect_pub = rospy.Publisher("/color_detection/update_detect", ColorsDetect, queue_size=1)
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.motion_manager = MotionManager(self.joints_pub)
        
        # 订阅颜色识别结果
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)

        rospy.Service('~enter', Empty, self.enter_func)  # 进入玩法
        rospy.Service('~exit', Empty, self.exit_func)  # 退出玩法
        rospy.Service('~start', Empty, self.start_srv_callback)  # 开始玩法
        rospy.Service('~stop', Empty, self.stop_srv_callback)  # 停止玩法
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色
        
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
            # 通知颜色识别准备，此时只显示摄像头原画
            target_color = rospy.get_param('~color', 'black')
            self.enter_func(None)
            self.set_color_srv_callback(String(target_color))  # 设置识别黑色
            self.start_srv_callback(None)  # 开启颜色识别
            common.loginfo('start track %s lane' % target_color)
        
        self.run()

    def shutdown(self, signum, frame):
        with self.lock:
            self.running = False
            common.loginfo('%s shutdown' % self.name)

    def init_action(self):
        # 初始位置
        self.motion_manager.set_servos(200, ((23, self.head_pan_init), (24, self.head_tilt_init)))
        self.gait_manager.stop()
        rospy.sleep(0.2)

    def enter_func(self, msg):
        self.init_action()
        rospy.ServiceProxy('/color_detection/enter', Empty)()
        
        common.loginfo("%s enter" % self.name)
        
        return EmptyResponse()

    def exit_func(self, msg):
        self.stop_srv_callback(Empty())
        rospy.ServiceProxy('/color_detection/exit', Empty)()
        
        common.loginfo('%s exit' % self.name)

        return EmptyResponse()

    def set_color_srv_callback(self, msg):
        # 设置颜色
        param = ColorDetect()
        param.color_name = msg.data
        param.use_name = True
        param.detect_type = 'line'
        param.line_roi.up.y_min = self.roi[0][0]
        param.line_roi.up.y_max = self.roi[0][1]
        param.line_roi.up.x_min = self.roi[0][2]
        param.line_roi.up.x_max = self.roi[0][3]

        param.line_roi.center.y_min = self.roi[1][0]
        param.line_roi.center.y_max = self.roi[1][1]
        param.line_roi.center.x_min = self.roi[1][2]
        param.line_roi.center.x_max = self.roi[1][3]

        param.line_roi.down.y_min = self.roi[2][0]
        param.line_roi.down.y_max = self.roi[2][1]
        param.line_roi.down.x_min = self.roi[2][2]
        param.line_roi.down.x_max = self.roi[2][3]

        param.min_area = 1
        param.max_area = 160*120
        
        msg = ColorsDetect()
        msg.data = [param]
        self.detect_pub.publish(msg)
        
        common.loginfo('%s set_color' % self.name)
        
        return [True, 'set_color']

    def start_srv_callback(self, msg):
        # 开始巡线
        with self.lock: 
            rospy.ServiceProxy('/color_detection/start', Empty)()
            self.start = True

        common.loginfo('%s start' % self.name)
        
        return EmptyResponse()

    def stop_srv_callback(self, msg):
        # 停止巡线
        with self.lock:
            rospy.ServiceProxy('/color_detection/stop', Empty)()
            self.start = False
            self.detect_pub.publish(ColorsDetect())

        common.loginfo('%s stop' % self.name)
        
        return EmptyResponse()

    def get_color_callback(self, msg):
        # 获取颜色识别结果
        if msg.data != []:
            self.count_miss = 0
            self.center = msg.data[0]
        elif self.start:
            self.count_miss += 1
            rospy.sleep(0.01)
            if self.count_miss > 100:
                self.count_miss = 0
                self.center = None
                self.gait_manager.stop()
    
    def body_track_process(self, center):
        # 躯体追踪
        
        self.center = None
        # 左右根据x坐标值进行调整
        if abs(center.x - center.width/2) < 10:
            yaw_output = 0
        elif abs(center.x - center.width/2) < center.width/4:
            yaw_output = math.copysign(1, center.x - center.width/2) + \
                         misc.val_map(center.x - center.width/2,
                                      -center.width/4, center.width/4,
                                      self.yaw_range[0] + 1, self.yaw_range[1] - 1)
        else:
            yaw_output = math.copysign(self.yaw_range[1], center.x - center.width/2)
        
        # 转弯时前进步幅减小
        if abs(yaw_output) > 7: 
            x_output = 0.01
        else:
            x_output = 0.02
        # print(self.center.x, self.center.y, x_output, yaw_output) 
        if rospy.get_time() > self.time_stamp:
            if abs(yaw_output) < 5:
                self.gait_manager.move(1, x_output, 0, int(-yaw_output))
            else:
                self.gait_manager.move(2, x_output, 0, int(-yaw_output))
            self.time_stamp = rospy.get_time() + 0.1

    def run(self):
        while self.running:
            if self.center is not None and self.start:
                self.body_track_process(self.center)
            else:
                rospy.sleep(0.01)

        self.init_action()
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    VisualPatrol('visual_patrol')
