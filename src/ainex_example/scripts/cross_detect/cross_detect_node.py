  #!/usr/bin/env python3
# encoding: utf-8
# @data:2023/07/03
# @author:aiden
# 十字路口检测
import rospy
import signal
from std_msgs.msg import String
from ainex_sdk import common
from ainex_example.common import Common
from ainex_example.visual_patrol import VisualPatrol
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect, ColorsDetect

class CrossDetectNode(Common):
    raise_right_hand_action_name = 'raise_right_hand'
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
        self.count_cross = 0
        self.state = 'visual_patrol'
        self.objects_info = None
        self.image_process_size = [160, 120]
        self.head_pan_init = 500   # 左右舵机的初始值
        self.head_tilt_init = 250  # 上下舵机的初始值
        super().__init__(name, self.head_pan_init, self.head_tilt_init)

        self.visual_patrol = VisualPatrol(self.gait_manager)
        signal.signal(signal.SIGINT, self.shutdown)
        
        # 订阅颜色识别结果
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)
        self.detect_pub = rospy.Publisher("/color_detection/update_detect", ColorsDetect, queue_size=1)
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色

        if rospy.get_param('~start', True):
            # 通知颜色识别准备，此时只显示摄像头原画
            target_color = rospy.get_param('~color', 'black')
            self.enter_func(None)
            self.set_color_srv_callback(String(target_color))  # 设置识别黑色
            self.start_srv_callback(None)  # 开启颜色识别
            common.loginfo('start cross %s lane' % target_color)

    def shutdown(self, signum, frame):
        with self.lock:
            self.running = False
            common.loginfo('%s shutdown' % self.name)

    def set_color_srv_callback(self, msg):
        # 设置颜色
        cross_param = ColorDetect()
        cross_param.color_name = msg.data
        cross_param.use_name = True
        cross_param.detect_type = 'cross'
        cross_param.image_process_size = self.image_process_size
        cross_param.roi.y_min = int(self.image_process_size[1]/4)
        cross_param.roi.y_max = int(3*self.image_process_size[1]/4)
        cross_param.roi.x_min = 0
        cross_param.roi.x_max = int(self.image_process_size[0])
        cross_param.min_area = 10
        cross_param.max_area = self.image_process_size[0]*self.image_process_size[1]
        
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

        self.detect_pub.publish([cross_param, line_param])

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
                cross_data = None
                for object_info in self.objects_info:
                    if object_info.type == 'line':
                        line_data = object_info
                    if object_info.type == 'cross':
                        cross_data = object_info
                if line_data is not None and self.state == 'visual_patrol':
                    self.visual_patrol.process(line_data.x, line_data.width)
                    rospy.sleep(0.1)
                elif self.state == 'cross_detect':
                    self.gait_manager.disable()
                    self.motion_manager.runAction(self.raise_right_hand_action_name)
                    walking_param = self.gait_manager.get_gait_param()
                    walking_param['body_height'] = 0.015
                    walking_param['pelvis_offset'] = 5
                    walking_param['step_height'] = 0.025
                    walking_param['hip_pitch_offset'] = 20
                    walking_param['z_swap_amplitude'] = 0.006
                    self.gait_manager.set_step([400, 0.2, 0.025], 0.02, 0, 0, walking_param, 0, 4)
                    self.motion_manager.runAction('stand')
                    self.state = 'visual_patrol'
                if cross_data is not None:
                    self.visual_patrol.update_go_gait(x_max=0.01)
                    self.visual_patrol.update_turn_gait(x_max=0.01)
                    if max(cross_data.y, cross_data.left_point[1], cross_data.right_point[1]) > cross_data.height/2:
                        self.count_cross += 1
                        if self.count_cross > 2:
                            self.gait_manager.stop()
                            self.count_cross = 0
                            common.loginfo('cross detect')
                            self.state = 'cross_detect'
                            self.visual_patrol.update_go_gait(x_max=0.02)
                            self.visual_patrol.update_turn_gait(x_max=0.02)
                    else:
                        self.count_cross = 0
                else:
                    self.count_cross = 0
            else:
                rospy.sleep(0.01)

        self.init_action()
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    CrossDetectNode('cross_detect').run()
