#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/06/03
# @author:aiden
# 自动踢球
import math
import rospy
import signal
from threading import RLock
from std_msgs.msg import Float64, String
from std_srvs.srv import Empty, EmptyResponse
from ainex_interfaces.srv import SetString
from ainex_sdk import pid, misc, pid_track, common
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from ainex_kinematics.gait_manager import GaitManager
from hiwonder_servo_controllers.motion_manager import MotionManager
from ainex_interfaces.msg import ObjectsInfo, ColorDetect, ColorsDetect

class KickBall:
    left_shot_action_name = 'left_shot'
    right_shot_action_name = 'right_shot'
    
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.running = True
        self.start = False
        self.count_miss = 0
        self.start_index = 0
        self.body_time_stamp = 0
        self.head_time_stamp = 0
        self.start_find_ball = False
       
        self.lock = RLock()
        signal.signal(signal.SIGINT, self.shutdown)
        
        # 机器人步态库调用
        self.gait_manager = GaitManager()
        
        # 调用上位机生成的动作
        self.joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
        self.motion_manager = MotionManager(self.joints_pub, '/home/ubuntu/software/ainex_controller/ActionGroups')
        
        # 头部的pid追踪
        self.head_pan_range = [125, 875]  # 左右转动限制在这个范围， 125为右
        self.head_tilt_range = [250, 500]  # 上下限制在这个范围， 250为下
        self.rl_dis = None
        self.ud_dis = None
        self.pid_rl = pid.PID(0.16, 0.0, 0.001)
        self.pid_ud = pid.PID(0.16, 0.0, 0.001)
        self.head_pan_init = 500  # 左右舵机的初始值
        self.head_tilt_init = 300 # 上下舵机的初始值
        self.head_track = pid_track.PIDTrack(self.pid_rl, self.pid_ud, self.head_pan_range, self.head_tilt_range, self.head_pan_init, self.head_tilt_init)
        
        # 躯体的追踪参数
        self.x_stop = 0.55  # 躯体停止前进时物体在画面的y轴上的比值(如画面为160x120，则物体的纵坐标为120x0.75)
        self.yaw_stop = 60  # 躯体停止转动时头部左右舵机脉宽值和中位500的差值
        self.x_range = [0, 0.02]  # 前进脚步大小单位m
        self.yaw_range = [-10, 10]  # 转弯范围正负10度
        
        # 找球时头部经过的5个位置，左右舵机，上下舵机，时间ms
        # left_down, left_up, center_up, right_up, right_down
        self.find_ball_position = [[650, 300, 1000], 
                                   [650, 500, 1000], 
                                   [500, 500, 1000], 
                                   [350, 500, 1000],
                                   [350, 300, 1000]
                                   ]
        
        # 更新颜色识别参数
        self.detect_pub = rospy.Publisher("/color_detection/update_detect", ColorsDetect, queue_size=1)

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
        common.loginfo('%s init_finish'%self.name)

        if rospy.get_param('~start', True):
            # 通知颜色识别准备，此时只显示摄像头原画
            target_color = rospy.get_param('~color', 'blue')
            self.enter_func(None)
            self.set_color_srv_callback(String(target_color))  # 设置识别蓝色
            self.start_srv_callback(None)  # 开启颜色识别
            common.loginfo('start kick %s ball'%target_color)
        
        self.run()

    def shutdown(self, signum, frame):
        with self.lock:
            self.running = False

            common.loginfo('%s shutdown'%self.name)

    def init_action(self):
        # 初始位置
        self.motion_manager.set_servos(200, ((23, self.head_pan_init), (24, self.head_tilt_init)))
        self.gait_manager.stop()
        rospy.sleep(0.2)

    def enter_func(self, msg):
        self.init_action()
        rospy.ServiceProxy('/color_detection/enter', Empty)()
        
        common.loginfo("%s enter"%self.name)
        
        return EmptyResponse()

    def exit_func(self, msg):
        self.stop_srv_callback(Empty())
        rospy.ServiceProxy('/color_detection/exit', Empty)()
        
        common.loginfo('%s exit'%self.name)

        return EmptyResponse()

    def set_color_srv_callback(self, msg):
        # 设置追踪颜色
        param = ColorDetect()
        param.color_name = msg.data
        param.detect_type = 'circle'
        param.use_name = True
        param.min_area = 10
        param.max_area = 160*120
        self.detect_pub.publish([param])
        
        common.loginfo('%s set_color'%self.name)
        
        return [True, 'set_color']

    def start_srv_callback(self, msg):
        # 开始追踪
        with self.lock:
            rospy.ServiceProxy('/color_detection/start', Empty)()
            self.start = True

        common.loginfo('%s start'%self.name)
        
        return EmptyResponse()

    def stop_srv_callback(self, msg):
        # 停止追踪
        with self.lock:
            rospy.ServiceProxy('/color_detection/stop', Empty)()
            self.start = False
            self.detect_pub.publish(ColorsDetect())

        common.loginfo('%s stop'%self.name)
        
        return EmptyResponse()

    def get_color_callback(self, msg):
        # 获取颜色识别结果
        if msg.data != []:
            if 80 > msg.data[0].radius > 3 and self.start:
                self.rl_dis, self.ud_dis = self.head_track_process(msg.data[0])
    
    def head_track_process(self, center):
        # 头部追踪
        if abs(center.x - center.width/2) < 10:
            center.x = center.width/2
        if abs(center.y - center.height/2) < 10:
            center.y = center.height/2
        rl_dis, ud_dis = self.head_track.track(center.x, center.y, center.width/2, center.height/2)
        self.motion_manager.set_servos(20, ((23, int(rl_dis)), (24, int(ud_dis))))

        return rl_dis, ud_dis

    def body_track_process(self, rl_dis, ud_dis):
        # 躯体追踪
        self.rl_dis, self.ud_dis = None, None
        
        # 左右根据头部左右舵机位置值进行调整
        yaw_stop = 500 + math.copysign(self.yaw_stop, rl_dis - 500)
        if abs(rl_dis - yaw_stop) < 5:
            yaw_output = 0
        elif abs(rl_dis - yaw_stop) < 125:
            # print(self.rl_dis, yaw_stop, misc.val_map(self.rl_dis - yaw_stop, -125, 125, self.yaw_range[0] + 3, self.yaw_range[1] - 3))
            yaw_output = math.copysign(3, rl_dis - yaw_stop) + misc.val_map(rl_dis - yaw_stop, -125, 125, self.yaw_range[0] + 3, self.yaw_range[1] - 3)
        else:
            yaw_output = math.copysign(self.yaw_range[1], rl_dis - 500)
        
        # 前进根据头部上下舵机位置
        if abs(ud_dis - self.head_tilt_range[0]) < 30:
            x_output = 0
        elif abs(ud_dis) > 300:
            if abs(yaw_output) == self.yaw_range[1]:
                x_output = 0.01
            else:
                x_output = self.x_range[1]
        else:
            x_output = 0.005 + misc.val_map(ud_dis, self.head_tilt_range[0], 400, self.x_range[0], self.x_range[1] - 0.005)
       
        # print(yaw_output, x_output)
        if x_output > self.x_range[1]:
            x_output = self.x_range[1]
        # yaw_output = 0
        # x_output = 0
        if rospy.get_time() > self.body_time_stamp:
            # 不同的步态需要相应的延时 
            if abs(yaw_output) >= 3 or abs(x_output >= 0.005):
                if abs(yaw_output) == self.yaw_range[1] or abs(x_output) == 0.02:
                    self.gait_manager.move(1, round(x_output, 4), 0, int(yaw_output))
                    self.body_time_stamp = rospy.get_time() + 0.1
                else:
                    self.gait_manager.move(2, round(x_output, 4), 0, int(yaw_output))
                    self.body_time_stamp = rospy.get_time() + 0.1
        if yaw_output == 0 and x_output == 0:
            self.gait_manager.disable()  # 需要先关闭行走
            if rl_dis > 500:
                self.motion_manager.runAction(self.left_shot_action_name)  # 固定左脚踢
            else:
                self.motion_manager.runAction(self.right_shot_action_name)  # 固定左脚踢

    def find_ball_process(self):
        # 找球
        if rospy.get_time() > self.head_time_stamp:
            if self.start_index > len(self.find_ball_position) - 1:
                self.start_index = 0
            rl_dis = self.find_ball_position[self.start_index][0]
            ud_dis = self.find_ball_position[self.start_index][1]
            self.head_track.update_position(rl_dis, ud_dis)  # pid的输出值要跟着更新
            self.motion_manager.set_servos(self.find_ball_position[self.start_index][2], ((23, rl_dis), (24, ud_dis)))

            if rospy.get_time() > self.body_time_stamp:  # 右转
                self.gait_manager.move(2, 0, 0, 5)
                self.body_time_stamp = rospy.get_time() + 0.1
            self.head_time_stamp = rospy.get_time() + self.find_ball_position[self.start_index][2]/1000.0
            self.start_index += 1

    def run(self):
        while self.running:
            if self.rl_dis is not None and self.start:
                self.body_track_process(self.rl_dis, self.ud_dis)
                self.count_miss = 0
                self.start_find_ball = False
            elif self.start and self.rl_dis is None and not self.start_find_ball:
                self.count_miss += 1
                if self.count_miss > 100:
                    self.count_miss = 0
                    self.start_find_ball = True
                    self.start_index = 0
                rospy.sleep(0.01)
            elif self.start and self.rl_dis is None and self.start_find_ball:
                self.find_ball_process()
            else:
                rospy.sleep(0.01)

        self.init_action()
        self.stop_srv_callback(None)
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    KickBall('kick_ball')
