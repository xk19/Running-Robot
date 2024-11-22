#!/usr/bin/env python3
# coding:utf-8
import rospy
import time
import startdoor
import threading
import square_hole
import obstacle
import overcome_obstacle
import door
import cross_the_bridge
import kickBallOnly
import take_the_stairs
import end_door
import sys

sys.path.append("/home/lemon/catkin_ws/src/aelos_smart_ros")
from leju import base_action


def main():
    if startdoor.start_door():                 # 第一关，道闸
        print("道闸结束,进入下一关")

    if square_hole.main('green_hole_chest'):  # 第二关，过坑，green_hole_chest为过坑地板颜色
        print("过坑结束，进入下一关")

    if obstacle.obstacle():                    # 第三关，地雷阵
        print("地雷阵结束，进入下一关")

    if overcome_obstacle.baffle():             # 第四关，挡板障碍
        print("翻越障碍结束，进入下一关")

    if door.into_the_door():                   # 第五关，过窄门框
        print("门框结束，进入下一关")

    if cross_the_bridge.Greenbridge():         # 第六关，过独木桥
        print("独木桥结束，进入下一关")

    if kickBallOnly.kick_ball():               # 第七关，踢球
        print("踢球结束，进入下一关")

    if take_the_stairs.floor():                # 第八关，楼梯
        print("楼梯结束，进入下一关")

    if end_door.end_door():                    # 第九关，双开道闸
        print("结束")
    

if __name__ == '__main__':
    rospy.init_node('runningrobot')
    time.sleep(3)
    main()