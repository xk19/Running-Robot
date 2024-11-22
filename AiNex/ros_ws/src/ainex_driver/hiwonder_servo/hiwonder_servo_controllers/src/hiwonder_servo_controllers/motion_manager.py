#!/usr/bin/env python3
# encoding: utf-8
# Date:2023/06/04
# 串口舵机控制库
import os
import time
import sqlite3 as sql
from hiwonder_servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur

class MotionManager:
    runningAction = False
    stopRunning = False
    
    def __init__(self, joints_pub, action_path=None):
        self.joints_pub = joints_pub
        self.action_path = action_path
        time.sleep(0.2)

    def set_servos(self, duration, pos_s):
        '''
        控制多个舵机转动
        :param duration: 时间ms
        :param pos_s: 舵机id和位置
        '''
        msg = MultiRawIdPosDur(id_pos_dur_list=list(map(lambda x: RawIdPosDur(int(x[0]), int(x[1]), int(duration)), pos_s)))
        self.joints_pub.publish(msg)

    def stop_action_group(self):
        self.stopRunning = True

    def runAction(self, actNum):
        '''
        运行动作组，无法发送stop停止信号
        :param actNum: 动作组名字 ， 字符串类型
        :param times:  运行次数
        :return:
        '''
        if actNum is None and self.action_path is not None:
            return
        actNum = os.path.join(self.action_path, actNum + ".d6a")
        self.stopRunning = False
        if os.path.exists(actNum):
            if not self.runningAction:
                self.runningAction = True
                ag = sql.connect(actNum)
                cu = ag.cursor()
                cu.execute("select * from ActionGroup")
                while True:
                    act = cu.fetchone()
                    if self.stopRunning:
                        self.stopRunning = False                   
                        break
                    if act is not None:
                        data = []
                        for i in range(0, len(act) - 2, 1):
                            data.extend([(i + 1, act[2 + i])])
                        # print(data[0])
                        self.set_servos(act[1], tuple(data)) 
                        time.sleep(float(act[1])/1000.0)
                    else:   # 运行完才退出
                        break
                self.runningAction = False
                
                cu.close()
                ag.close()
        else:
            self.runningAction = False
            print('can not find aciton file')


if __name__ == '__main__':
    import rospy
    from hiwonder_servo_msgs.msg import MultiRawIdPosDur
    
    rospy.init_node('motion_manager')
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    motion_manager = MotionManager(joints_pub, '/home/ubuntu/software/ainex_controller/ActionGroups')
    
    # 单个舵机运行
    motion_manager.set_servos(500, ((23, 300), ))
    time.sleep(0.5) 
    # 多个舵机运行
    motion_manager.set_servos(500, ((23, 500), (24, 500)))
    time.sleep(0.5)
    # 执行动作组
    motion_manager.run_action('left_shot')
    motion_manager.run_action('right_shot')
