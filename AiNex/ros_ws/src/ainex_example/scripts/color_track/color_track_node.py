#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/06/01
# @author:aiden
# 颜色追踪
import math
import rospy
import signal
from threading import RLock
import ainex_sdk.pid as pid
from ainex_sdk import common
from ainex_sdk.pid_track import PIDTrack
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
from hiwonder_servo_msgs.msg import CommandDuration
from ainex_interfaces.srv import SetString
from ainex_interfaces.msg import ObjectsInfo, ColorDetect, ColorsDetect

class ColorTrackNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.center = None
        self.running = True
        self.start = False
        
        self.lock = RLock()
        signal.signal(signal.SIGINT, self.shutdown)

        head_pan_range = [math.radians(-90), math.radians(90)]
        head_tilt_range = [math.radians(-45), math.radians(30)]
        self.pid_rl = pid.PID(0.00055, 0.0, 0.00001)
        self.pid_ud = pid.PID(0.00055, 0.0, 0.00001)
        self.track = PIDTrack(self.pid_rl, self.pid_ud, head_pan_range, head_tilt_range)

        self.head_pan_pub = rospy.Publisher("/head_pan_controller/command_duration", CommandDuration, queue_size=1)
        self.head_tilt_pub = rospy.Publisher("/head_tilt_controller/command_duration", CommandDuration, queue_size=1)
        self.detect_pub = rospy.Publisher("/color_detection/update_detect", ColorsDetect, queue_size=1)
        rospy.Subscriber('/object/pixel_coords', ObjectsInfo, self.get_color_callback)

        rospy.Service('~enter', Empty, self.enter_func)  # 进入玩法
        rospy.Service('~exit', Empty, self.exit_func)  # 退出玩法
        rospy.Service('~start', Empty, self.start_srv_callback)  # 开始玩法
        rospy.Service('~stop', Empty, self.stop_srv_callback)  # 停止玩法
        rospy.Service('~set_color', SetString, self.set_color_srv_callback)  # 设置颜色

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
            target_color = rospy.get_param('~color', 'blue')
            self.enter_func(None)
            self.set_color_srv_callback(String(target_color))
            self.start_srv_callback(None)
            common.loginfo('start track %s'%target_color)

        self.run()

    def shutdown(self, signum, frame):
        with self.lock:
            self.running = False
            common.loginfo('%s shutdown'%self.name)

    def init_action(self):
        self.head_pan_pub.publish(CommandDuration(data=0, duration=500))
        self.head_tilt_pub.publish(CommandDuration(data=0, duration=500))

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
        param = ColorDetect()
        param.color_name = msg.data
        param.use_name = True
        param.detect_type = 'circle'
        param.min_area = 10
        param.max_area = 160*120
        self.detect_pub.publish([param])

        common.loginfo('%s set_color'%self.name)
        
        return [True, 'set_color']

    def start_srv_callback(self, msg):
        with self.lock:
            rospy.ServiceProxy('/color_detection/start', Empty)()
            self.start = True
            self.init_action()
        
        common.loginfo('%s start'%self.name)
        
        return EmptyResponse()

    def stop_srv_callback(self, msg):
        with self.lock:
            rospy.ServiceProxy('/color_detection/stop', Empty)()
            self.start = False
            self.detect_pub.publish(ColorsDetect())
            self.track.update_position(0, 0)
            self.track.clear()
            self.init_action()
            
        common.loginfo('%s stop'%self.name)
        
        return EmptyResponse()

    def get_color_callback(self, msg):
        if msg.data != []:
            self.center = msg.data[0]

    def head_track_process(self, center):
        rl_dis, ud_dis = self.track.track(center.x, center.y, center.width/2, center.height/2)
        self.head_pan_pub.publish(CommandDuration(data=rl_dis, duration=20))
        self.head_tilt_pub.publish(CommandDuration(data=ud_dis, duration=20))
        rospy.sleep(0.02)

    def run(self):
        while self.running:
            if self.center is not None and self.start:
                self.head_track_process(self.center)
                self.center = None
            else:
                rospy.sleep(0.01)

        self.init_action()
        rospy.signal_shutdown('shutdown')

if __name__ == "__main__":
    ColorTrackNode('color_track')
