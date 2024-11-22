#!/usr/bin/env python3
# coding:utf-8

import numpy as np
import cv2
import math
import threading
import time
import rospy
import sys

from image_converter import ImgConverter

sys.path.append("/home/lemon/catkin_ws/src/aelos_smart_ros")
from leju import base_action

##################################动作执行####################################
def action_append(act_name):
    print(f'执行动作: {act_name}')
    # time.sleep(1)
    base_action.action(act_name)

chest_r_width = 480
chest_r_height = 640
head_r_width = 640
head_r_height = 480
chest_ret = False     # 读取图像标志位
ret = False           # 读取图像标志位
ChestOrg_img = None   # 原始图像更新
HeadOrg_img = None    # 原始图像更新
ChestOrg_copy = None
HeadOrg_copy = None
r_width = 480
r_height = 640
img_debug = True



################################################读取图像线程#################################################

def get_chest_img():
    global ChestOrg_img, chest_ret,image_reader
    image_reader = ImgConverter()
    while True:
        chest_ret, ChestOrg_img = image_reader.chest_image()
        #print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
        #print(ChestOrg_img)
        time.sleep(0.05)

# 读取图像线程
th1 = threading.Thread(target=get_chest_img)
th1.setDaemon(True)
th1.start()




# 得到最大轮廓和对应的最大面积 
def getAreaMaxContour1(contours):    # 返回轮廓 和 轮廓面积
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 25:  #只有在面积大于25时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
    return area_max_contour, contour_area_max  # 返回最大的轮廓

color_range = {
    'green_hole_chest':[(67 , 60 , 106), (90 , 249 , 255)],
    'blue_hole_chest':[(102 , 123 , 132), (110 , 213 , 235)],
}

#根据颜色边缘调整角度与位置（胸部）
def edge_angle_chest(color):
    global org_img, state, state_sel, step, reset, skip, debug   
    r_w = chest_r_width
    r_h = chest_r_height
    top_angle = 0
    T_B_angle = 0
    topcenter_x = 0.5 * r_w
    topcenter_y = 0
    bottomcenter_x = 0.5 * r_w
    bottomcenter_y = 0
    while(True):
        step = 0
        OrgFrame = ChestOrg_img.copy()

        # 初始化 bottom_right  bottom_left
        bottom_right = (480,0)
        bottom_left =  (0,0)
        top_right = (480,0)  # 右上角点坐标
        top_left = (0,0)  # 左上角点坐标

        frame = cv2.resize(OrgFrame, (chest_r_width, chest_r_height), interpolation=cv2.INTER_LINEAR)
        frame_copy = frame.copy()
        # 获取图像中心点坐标x, y
        center = []
        # 开始处理图像
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (3, 3), 0)
        Imask = cv2.inRange(hsv, color_range[color][0], color_range[color][1])
        Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)

        cnts, hierarchy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
        
        cnt_sum, area_max = getAreaMaxContour1(cnts)  # 找出最大轮廓
        C_percent = round(area_max * 100 / (r_w * r_h), 2)  # 最大轮廓百分比
        cv2.drawContours(frame, cnt_sum, -1, (255, 0, 255), 3)

        if cnt_sum is not None:
            see = True
            rect = cv2.minAreaRect(cnt_sum)#最小外接矩形
            box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
            
            bottom_right = cnt_sum[0][0]  # 右下角点坐标
            bottom_left = cnt_sum[0][0]  # 左下角点坐标
            top_right = cnt_sum[0][0]  # 右上角点坐标
            top_left = cnt_sum[0][0]  # 左上角点坐标
            for c in cnt_sum:

                if c[0][0] + 1 * (r_h - c[0][1]) < bottom_left[0] + 1 * (r_h - bottom_left[1]):
                    bottom_left = c[0]
                if c[0][0] + 1 * c[0][1] > bottom_right[0] + 1 * bottom_right[1]:
                    bottom_right = c[0]

                if c[0][0] + 3 * c[0][1] < top_left[0] + 3 * top_left[1]:
                    top_left = c[0]
                if (r_w - c[0][0]) + 3 * c[0][1] < (r_w - top_right[0]) + 3 * top_right[1]:
                    top_right = c[0]

                # if debug:
                #     handling = ChestOrg_img.copy()
                #     cv2.circle(handling, (c[0][0], c[0][1]), 5, [0, 255, 0], 2)
                #     cv2.circle(handling, (bottom_left[0], bottom_left[1]), 5, [255, 255, 0], 2)
                #     cv2.circle(handling, (bottom_right[0], bottom_right[1]), 5, [255, 0, 255], 2)
                #     cv2.imshow('handling', handling)  # 显示图像
                #     cv2.waitKey(2)

            bottomcenter_x = (bottom_left[0] + bottom_right[0]) / 2  # 得到bottom中心坐标
            bottomcenter_y = (bottom_left[1] + bottom_right[1]) / 2

            topcenter_x = (top_right[0] + top_left[0]) / 2  # 得到top中心坐标
            topcenter_y = (top_left[1] + top_right[1]) / 2

            bottom_angle =  -math.atan( (bottom_right[1]-bottom_left[1]) / (bottom_right[0]-bottom_left[0]) ) *180.0/math.pi
            top_angle =  -math.atan( (top_right[1]-top_left[1]) / (top_right[0]-top_left[0]) ) *180.0/math.pi
            if math.fabs(topcenter_x - bottomcenter_x) <= 1:  # 得到连线的角度
                T_B_angle = 90
            else:
                T_B_angle = - math.atan((topcenter_y - bottomcenter_y) / (topcenter_x - bottomcenter_x)) * 180.0 / math.pi

            if img_debug:
                cv2.drawContours(frame_copy, [box], 0, (0, 255, 0), 2)  # 将大矩形画在图上
                cv2.line(frame_copy, (bottom_left[0],bottom_left[1]), (bottom_right[0],bottom_right[1]), (255, 255, 0), thickness=2)
                cv2.line(frame_copy, (top_left[0],top_left[1]), (top_right[0],top_right[1]), (255, 255, 0), thickness=2)
                cv2.line(frame_copy, (int(bottomcenter_x),int(bottomcenter_y)), (int(topcenter_x),int(topcenter_y)), (255, 255, 255), thickness=2)    # T_B_line

                cv2.putText(frame_copy, "bottom_angle:" + str(bottom_angle), (30, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                cv2.putText(frame_copy, "top_angle:" + str(top_angle),(30, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
                cv2.putText(frame_copy, "T_B_angle:" + str(T_B_angle),(30, 400), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

                cv2.putText(frame_copy, "bottomcenter_x:" + str(bottomcenter_x), (30, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                cv2.putText(frame_copy, "y:" + str(int(bottomcenter_y)), (300, 480), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR

                cv2.putText(frame_copy, "topcenter_x:" + str(topcenter_x), (30, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                cv2.putText(frame_copy, "topcenter_y:" + str(int(topcenter_y)), (230, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.65,(0, 0, 0), 2)  # (0, 0, 255)BGR

                cv2.putText(frame_copy, 'C_percent:' + str(C_percent) + '%', (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)
                cv2.putText(frame_copy, "step:" + str(step), (30, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),2)  # (0, 0, 255)BGR
                
                cv2.circle(frame_copy, (int(topcenter_x), int(topcenter_y)), 5, [255, 0, 255], 2)
                cv2.circle(frame_copy, (int(bottomcenter_x), int(bottomcenter_y)), 5, [255, 0, 255], 2)
                cv2.circle(frame_copy, (top_right[0], top_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(frame_copy, (top_left[0], top_left[1]), 5, [0, 255, 255], 2)
                cv2.circle(frame_copy, (bottom_right[0], bottom_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(frame_copy, (bottom_left[0], bottom_left[1]), 5, [0, 255, 255], 2)
                # cv2.imshow('Chest_Camera', frame_copy)  # 显示图像
                cv2.imwrite('./Chest_Camera.jpg', frame_copy)        #查看识别情况

        else:
            print("815L  chest NONE")



        # 决策执行动作

        if step == 0:   # 前进依据chest 调整大致位置，方向  看底边线调整角度
        
            if top_angle > 2.5:  # 需要左转
                if top_angle > 6:
                    print("826L 大左转一下  turn001L ",top_angle)
                    action_append("turn001L")
                else:
                    print("829L bottom_angle > 3 需要小左转 turn001L ",top_angle)
                    action_append("turn001L")
            elif top_angle < -2.5:  # 需要右转
                if top_angle < -6:
                    # print("833L 右大旋转  turn001R < -6 ",Head_L_R_angle)
                    action_append("turn001R")
                else:
                    print("836L bottom_angle < -3 需要小右转 turn001R ",top_angle)
                    action_append("turn001R")
            elif -2.5 <= top_angle <= 2.5:  # 角度正确
                print("839L 角度合适")

                if topcenter_x > 255 or topcenter_x < 230:
                    if topcenter_x > 255:
                        print("843L 微微右移,",topcenter_x)
                        action_append("Right3move")
                    elif topcenter_x < 230:
                        print("846L 微微左移,",topcenter_x)
                        action_append("Left3move")

                else:
                    print("850L 位置合适")
                    break





#过坑识别
def hole_recognize(color):
    global org_chest_img
    src = ChestOrg_img.copy()
    Area = 0
    src = src[int(100):int(400),int(50):int(500)]
    src = cv2.GaussianBlur(src, (5, 5), 0)
    hsv_img = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, color_range[color][0], color_range[color][1])
    closed = cv2.dilate(mask, None, iterations=5)
    closed = cv2.erode(closed, None, iterations=8)


    contours, hierarchy = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) > 0:
        max_area = max(contours, key=cv2.contourArea)
        Area = cv2.contourArea(max_area)
        rect = cv2.minAreaRect(max_area)
        #print(rect[0])
        # # print(Area)
    contours2, hierarchy2 = cv2.findContours(closed, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

    if Area > 20000 and len(contours2) >= 2:
        return True
    else:
        return False



        ###################### 过            坑-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-
def hole_edge(color,action_append):
    edge_angle_chest(color)#调整好角度与距离
    while(1):
        src = ChestOrg_img.copy()
        src = src[int(100):int(400),int(50):int(500)]
        src = cv2.GaussianBlur(src, (5, 5), 0)
        hsv_img = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, color_range[color][0], color_range[color][1])


        mask2 = cv2.erode(mask, None, iterations=5)
        mask1 = cv2.dilate(mask2, None, iterations=8)

        contours2, hierarchy2 = cv2.findContours(mask1, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        # dts = cv2.drawContours(ChestOrg_img.copy(), contours2, 0,(0, 0, 255),cv2.FILLED)
        # cv2.imwrite('./test.jpg', dts)
        if len(contours2) >= 2:
            print("3146L 仍然看得到内轮廓，向前走 forwardSlow0403")
            action_append("forwardSlow0403")

        else:
            print("已近迈进，正式进入过坑阶段")
            action_append("Stand")
            if  color == 'blue_hole_chest':
                hole_edge_main('blue_hole_chest',action_append)
                break
            elif color == 'green_hole_chest':
                hole_edge_main('green_hole_chest',action_append)
                break
        
def hole_edge_main(color):
    global HeadOrg_img,chest_copy, reset, skip,handling,ChestOrg_img
    global handling
    angle = 90
    see = False
    headTURN = 0

    step = 1
    print("hole edge")
    while True:
        OrgFrame = ChestOrg_img.copy()
        x_start = 260
        blobs = OrgFrame[int(0):int(480), int(x_start):int(380)]  # 只对中间部分识别处理  Y , X
        handling = blobs.copy()
        frame_mask = blobs.copy()

        # 获取图像中心点坐标x, y
        center = []
        # 开始处理图像
        hsv = cv2.cvtColor(frame_mask, cv2.COLOR_BGR2HSV)
        # hsv = cv2.GaussianBlur(hsv, (3, 3), 0)
        hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
        cv2.imwrite('./hsv.jpg', hsv)        #查看识别情况
        Imask = cv2.inRange(hsv, color_range[color][0], color_range[color][1])
        # Imask = cv2.erode(Imask, np.ones((3, 3), np.uint8), iterations=1)
        Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=3)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
        Imask = cv2.morphologyEx(Imask, cv2.MORPH_OPEN, kernel)
        contours, hierarchy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
        # cv2.imwrite('./Imask.jpg', Imask)        #查看识别情况
        if len(contours) > 0: 
            max_area = max(contours, key=cv2.contourArea)
            epsilon = 0.05 * cv2.arcLength(max_area, True)
            approx = cv2.approxPolyDP(max_area, epsilon, True)
            approx_list = list(approx)
            approx_after = []
            for i in range(len(approx_list)):
                approx_after.append(approx_list[i][0])
            approx_sort = sorted(approx_after, key=lambda x: x[1], reverse=True)
            if len(approx_sort) == 4:
                bottom_line = (approx_sort[3], approx_sort[2])
                center_x = (bottom_line[1][0]+bottom_line[0][0])/2
                center_y = (bottom_line[1][1]+bottom_line[0][1])/2
            else:
                bottom_line = None

        else:
            bottom_line = None
            
        # 初始化
        L_R_angle = 0 
        blackLine_L = [0,0]
        blackLine_R = [0,0]

        if bottom_line is not None:
            see = True
            if bottom_line[0][1] - bottom_line[1][1]==0:
                angle=90
            else:
                angle = - math.atan((bottom_line[1][1] - bottom_line[0][1]) / (bottom_line[1][0] - bottom_line[0][0]))*180.0/math.pi
            Ycenter = int((bottom_line[1][1] + bottom_line[0][1]) / 2)
            Xcenter = int((bottom_line[1][0] + bottom_line[0][0]) / 2)
            if bottom_line[1][1] > bottom_line[0][1]:
                blackLine_L = [bottom_line[1][0] , bottom_line[1][1]]
                blackLine_R = [bottom_line[0][0] , bottom_line[0][1]]
            else:
                blackLine_L =  [bottom_line[0][0] , bottom_line[0][1]]
                blackLine_R = [bottom_line[1][0] , bottom_line[1][1]]
            cv2.circle(OrgFrame, (Xcenter + x_start, Ycenter), 10, (255,255,0), -1)#画出中心点

            if blackLine_L[0] == blackLine_R[0]:
                L_R_angle = 0
            else:
                L_R_angle =  (-math.atan( (blackLine_L[1]-blackLine_R[1]) / (blackLine_L[0]-blackLine_R[0]) ) *180.0/math.pi)-4


            print("529L_R_angle = ",L_R_angle)
            if img_debug:
                
                cv2.circle(OrgFrame, (blackLine_L[0] + x_start, blackLine_L[1]), 5, [0, 255, 255], 2)
                cv2.circle(OrgFrame, (blackLine_R[0] + x_start, blackLine_R[1]), 5, [255, 0, 255], 2)
                cv2.line(OrgFrame, (blackLine_R[0] + x_start,blackLine_R[1]), (blackLine_L[0] + x_start,blackLine_L[1]), (0, 255, 255), thickness=2)
                cv2.putText(OrgFrame, "L_R_angle:" + str(L_R_angle),(10, OrgFrame.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(OrgFrame, "Xcenter:" + str(Xcenter + x_start),(10, OrgFrame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
                cv2.putText(OrgFrame, "Ycenter:" + str(Ycenter),(200, OrgFrame.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

                cv2.imwrite('./frame_mask.jpg', frame_mask)        #查看识别情况
                # cv2.imshow('black', Imask)
                # cv2.imshow('OrgFrame', OrgFrame)
                cv2.imwrite('./OrgFrame.jpg', OrgFrame)        #查看识别情况
                # cv2.waitKey(10)
        else:
            see = False
        time.sleep(1)

     # 决策执行动作
        if step == 1:
            if not see:  # not see the edge
                # cv2.destroyAllWindows()
                print("3273L 右侧看不到边缘 左侧移 Left3move")
                action_append("Left3move")
            else:   # 0
                if L_R_angle > 3:
                    if L_R_angle > 7:
                        headTURN += 1
                        print("3279L 左da旋转 turn001L ",L_R_angle)
                        action_append("turn001L")

                    else:
                        print("3283L 左旋转 turn000L ",L_R_angle)
                        headTURN += 1
                        action_append("turn001L")

                    
                elif L_R_angle < -6:
                    if L_R_angle < -8:
                        headTURN += 1
                        print("3292L 右da旋转  turn001R ",L_R_angle)
                        action_append("turn001R")

                    else:
                        print("3296L 右旋转  turn000R ",L_R_angle)
                        action_append("turn001R")

                    
                elif Xcenter <= 63:
                    print("3306L 左侧移 Left02move <= 63 ",Xcenter)
                    action_append("Left02move")
                elif Xcenter >= 70:
                    print("3309L 右侧移 Right02move >= 70 ",Xcenter)
                    action_append("Right02move")
                else:
                    print("3312L 右看 X位置ok")
                    action_append("Left02move")   #先左移一步
                    action_append("fastForward03")
                    print("向前一步")
                    action_append("forwardSlow0403")
                    action_append("forwardSlow0403")  
                    action_append("Forwalk00") 
                    action_append("Stand")
                    step = 2
                 

        elif step == 2:
            if not see:  # not see the edge
                # cv2.destroyAllWindows()
                print("3327L 看不到边缘 右侧移 Right02move")
                # action_append("Right02move")
                step == 3
            else:   # 0
                if L_R_angle > 3:
                    if L_R_angle > 7:
                        print("3332L 左旋转 turn001L ",L_R_angle)
                        action_append("turn001L")
                    else:
                        print("3335L 左旋转 turn001L ",L_R_angle)
                        action_append("turn001L")
                    
                elif L_R_angle < -6:
                    if L_R_angle < -8:
                        print("3341L 右旋转  turn001R ",L_R_angle)
                        action_append("turn001R")
                    else:
                        print("3344L 右旋转  turn001R ",L_R_angle)
                        action_append("turn001R")
                else:
                    print("666L 右看 X位置ok")
                    step = 3
        
        elif step == 3:
            print("3352L 右侧看到绿色边缘 右侧移 Right3move")
            action_append("Right3move")
            action_append("Right3move")
            action_append("Right3move")
            step = 5

        elif step == 5:
            print("过坑阶段结束")
            break


def main(color):
    time.sleep(3)
    while True:
        recognize = hole_recognize(color)
        print("recognize = ",recognize)
        if recognize:
            hole_edge(color,action_append)
            break
        time.sleep(0.5)
    return True



if __name__ == '__main__':
    rospy.init_node('square')
    main('green_hole_chest')