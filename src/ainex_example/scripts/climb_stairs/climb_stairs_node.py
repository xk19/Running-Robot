#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/04
# @author:aiden
# 上阶梯or下阶梯
import rospy
import signal
from ainex_sdk import common
from std_msgs.msg import String
from ainex_example.common import Common
from ainex_example.approach_object import ApproachObject
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect

class ClimbStairsNode(Common):
    climb_stairs_action_name = 'climb_stairs'
    descend_stairs_action_name = 'descend_stairs'
    
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True
        self.object_info = None
        self.state = 'climb_stairs'
        self.image_process_size = [160, 120]
        self.head_pan_init = 500   # 左右舵机的初始值
        self.head_tilt_init = 260  # 上下舵机的初始值
        super().__init__(name, self.head_pan_init, self.head_tilt_init)
        
        if self.state == 'climb_stairs':
            self.approach_object = ApproachObject(self.gait_manager, 0.68, 0.5, 1)
        else:
            self.approach_object = ApproachObject(self.gait_manager, 0.76, 0.5, 1)
        signal.signal(signal.SIGINT, self.shutdown)

        # 订阅颜色识别结果
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色
        self.motion_manager.runAction('hand_back')
        if rospy.get_param('~start', True):
            # 通知颜色识别准备，此时只显示摄像头原画
            target_color = rospy.get_param('~color', 'red')
            self.enter_func(None)
            self.set_color_srv_callback(String(target_color))  # 设置识别红色
            self.start_srv_callback(None)  # 开启颜色识别
            common.loginfo('start climb_stairs %s' % target_color)

    def shutdown(self, signum, frame):
        self.running = False

        common.loginfo('%s shutdown' % self.name)

    def set_color_srv_callback(self, msg):
        # 设置追踪颜色
        param = ColorDetect()
        param.color_name = msg.data
        param.detect_type = 'side'
        param.use_name = True
        param.image_process_size = self.image_process_size
        param.roi.y_min = int(self.image_process_size[1]/5)
        param.roi.y_max = int(self.image_process_size[1])
        param.roi.x_min = 0
        param.roi.x_max = int(self.image_process_size[0])
        param.min_area = 10
        param.max_area = self.image_process_size[0]*self.image_process_size[1]
        self.detect_pub.publish([param])
        
        common.loginfo('%s set_color' % self.name)
        
        return [True, 'set_color']

    def get_color_callback(self, msg):
        # 获取颜色识别结果
        if msg.data != []:
            self.object_info = msg.data[0]

    def run(self):
        while self.running:
            if self.object_info is not None and self.start:
                if self.approach_object.process(self.object_info.x, max(self.object_info.y, self.object_info.left_point[1], self.object_info.right_point[1]), self.object_info.angle, self.object_info.width, self.object_info.height):
                    walking_param = self.gait_manager.get_gait_param()
                    walking_param['pelvis_offset'] = 5
                    walking_param['step_height'] = 0.02
                    walking_param['z_swap_amplitude'] = 0.006
                    self.gait_manager.set_step([400, 0.2, 0.02], 0.005, 0, 0, walking_param, arm_swap=0, step_num=2)
                    self.gait_manager.disable()
                    if self.state == 'climb_stairs':
                        common.loginfo('climb_stairs')
                        self.motion_manager.runAction(self.climb_stairs_action_name)
                    else:
                        common.loginfo('descend_stairs')
                        self.motion_manager.runAction(self.descend_stairs_action_name)
                    self.running = False
            else:
                rospy.sleep(0.01)

        self.init_action()
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    ClimbStairsNode('climb_stairs').run()
