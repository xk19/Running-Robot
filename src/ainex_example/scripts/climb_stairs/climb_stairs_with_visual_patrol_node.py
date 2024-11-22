#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/04
# @author:aiden
# 巡线上下阶梯
import rospy
import signal
from std_msgs.msg import Float64, String
from ainex_sdk import misc, common
from ainex_example.common import Common
from ainex_example.visual_patrol import VisualPatrol
from ainex_example.approach_object import ApproachObject
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect, ROI

class ClimbStairsVisualPatrol(Common):
    # 按顺序检测三个roi，如果检测到黑线立刻跳出
    roi = [ # [ROI, weight]
            (5/12, 6/12, 1/4, 3/4),
            (6/12, 7/12, 1/4, 3/4),
            (7/12, 8/12, 1/4, 3/4)
          ]
    climb_stairs_action_name = 'climb_stairs'
    descend_stairs_action_name = 'descend_stairs'
    
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True
        self.count_miss = 0
        self.objects_info = None
        self.state = "visual_patrol"
        self.image_process_size = [160, 120]
        self.head_pan_init = 500  # 左右舵机的初始值
        self.head_tilt_init = 260 # 上下舵机的初始值
        super().__init__(name, self.head_pan_init, self.head_tilt_init)

        self.visual_patrol = VisualPatrol(self.gait_manager)
        self.approach_object = ApproachObject(self.gait_manager, 0.65, 0.5, 1)
        signal.signal(signal.SIGINT, self.shutdown)

        # 订阅颜色识别结果
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色

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
        stairs_param = ColorDetect()
        stairs_param.color_name = msg.data
        stairs_param.detect_type = 'side'
        stairs_param.use_name = True
        stairs_param.image_process_size = self.image_process_size
        stairs_param.roi.y_min = int(self.image_process_size[1]/5)
        stairs_param.roi.y_max = int(self.image_process_size[1])
        stairs_param.roi.x_min = 0
        stairs_param.roi.x_max = int(self.image_process_size[0])
        stairs_param.min_area = 10
        stairs_param.max_area = self.image_process_size[0]*self.image_process_size[1]

        line_param = ColorDetect()
        line_param.color_name = 'black'
        line_param.use_name = True
        line_param.detect_type = 'line'
        line_param.image_process_size = self.image_process_size
        line_param.line_roi.up.y_min = int(self.roi[0][0] * self.image_process_size[1])
        line_param.line_roi.up.y_max = int(self.roi[0][1] * self.image_process_size[1])
        line_param.line_roi.up.x_min = int(self.roi[0][2] * self.image_process_size[0])
        line_param.line_roi.up.x_max = int(self.roi[0][3] * self.image_process_size[0])

        line_param.line_roi.center.y_min = int(self.roi[1][0] * self.image_process_size[1])
        line_param.line_roi.center.y_max = int(self.roi[1][1] * self.image_process_size[1])
        line_param.line_roi.center.x_min = int(self.roi[1][2] * self.image_process_size[0])
        line_param.line_roi.center.x_max = int(self.roi[1][3] * self.image_process_size[0])

        line_param.line_roi.down.y_min = int(self.roi[2][0] * self.image_process_size[1])
        line_param.line_roi.down.y_max = int(self.roi[2][1] * self.image_process_size[1])
        line_param.line_roi.down.x_min = int(self.roi[2][2] * self.image_process_size[0])
        line_param.line_roi.down.x_max = int(self.roi[2][3] * self.image_process_size[0])

        line_param.min_area = 1
        line_param.max_area = self.image_process_size[0] * self.image_process_size[1]

        self.detect_pub.publish([stairs_param, line_param])
        
        common.loginfo('%s set_color' % self.name)
        
        return [True, 'set_color']

    def get_color_callback(self, msg):
        # 获取颜色识别结果
        if msg.data != []:
            self.count_miss = 0
            self.objects_info = msg.data
        elif self.start:
            self.count_miss += 1
            rospy.sleep(0.01)
            if self.count_miss > 100:
                self.count_miss = 0
                self.objects_info = None

    def run(self):
        while self.running:
            if self.objects_info is not None and self.start:
                line_data = None
                stairs_data = None
                for object_info in self.objects_info:
                    if object_info.type == 'line':
                        line_data = object_info
                    if object_info.type == 'side':
                        stairs_data = object_info

                if line_data is not None and (self.state == 'visual_patrol' or self.state == 'climb_stairs_finish' or self.state == 'decrease_stairs_finish'):
                    self.visual_patrol.process(line_data.x, line_data.width)
                    if stairs_data is not None:
                        if self.state != 'climb_stairs_finish':
                            if max(stairs_data.y, stairs_data.left_point[1], stairs_data.right_point[1]) > stairs_data.height/4:
                                common.loginfo('climb_stairs approach')
                                self.state = 'climb_stairs'
                                self.gait_manager.disable()
                                self.motion_manager.runAction('hand_back')  # 手往后，防止遮挡
                                rospy.sleep(0.5)
                        else:
                            if max(stairs_data.y, stairs_data.left_point[1], stairs_data.right_point[1]) > stairs_data.height/2:
                                common.loginfo('decrease_stairs approach')
                                self.state = 'decrease_stairs'
                                self.gait_manager.disable()
                                self.motion_manager.runAction('hand_back')  # 手往后，防止遮挡
                                rospy.sleep(0.5)
                if stairs_data is not None:
                    if self.state != 'visual_patrol':
                        if self.approach_object.process(stairs_data.x, max(stairs_data.y, stairs_data.left_point[1], stairs_data.right_point[1]), stairs_data.angle, stairs_data.width, stairs_data.height):
                            walking_param = self.gait_manager.get_gait_param()
                            walking_param['pelvis_offset'] = 5
                            walking_param['step_height'] = 0.02
                            walking_param['z_swap_amplitude'] = 0.006
                            self.gait_manager.set_step([400, 0.2, 0.02], 0.005, 0, 0, walking_param, arm_swap=0, step_num=2)
                            self.gait_manager.disable()
                            if self.state == 'climb_stairs':
                                common.loginfo('climb_stairs')
                                self.motion_manager.runAction(self.climb_stairs_action_name)
                                self.state = 'climb_stairs_finish'
                                self.approach_object = ApproachObject(self.gait_manager, 0.76, 0.5, 1)  # 和上台阶接近位置不一样
                            elif self.state == 'decrease_stairs':
                                common.loginfo('decrease_stairs')
                                self.motion_manager.runAction(self.descend_stairs_action_name)
                                self.state = 'decrease_stairs_finish'
                                self.approach_object = ApproachObject(self.gait_manager, 0.68, 0.5, 1)
            else:
                rospy.sleep(0.01)
        self.init_action()
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    ClimbStairsVisualPatrol('climb_stairs_visual_patrol').run()
