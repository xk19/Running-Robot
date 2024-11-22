#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/06/03
# @author:aiden
# 识别颜色以及计算物体的位置信息
import cv2
import math
import numpy as np

class ColorDetection:
    def __init__(self, lab_config, detect_info, image_proc_size=[160, 120]):
        '''
        lab_config: lab阈值，字典形式
        detect_info: 检测类型，ros格式
        image_proc_size: 图像处理大小
        '''
        self.lab_data = lab_config
        self.detect_info = detect_info.data
        
        # 图像缩放加速识别
        self.image_proc_size = image_proc_size
        
    def update_lab_config(self, lab_data):
        '''
        更新参数
        :param lab_data:
        :return:
        '''
        self.lab_data = lab_data

    def update_detect_info(self, detect_info):
        self.detect_info = detect_info
    
    def erode_and_dilate(self, binary, kernel=3):
        # 腐蚀膨胀
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel, kernel))
        eroded = cv2.erode(binary, element)  # 腐蚀
        dilated = cv2.dilate(eroded, element)  # 膨胀

        return dilated

    def point_remapped(self, point, now, new, data_type=float):
        """
        将一个点的坐标从一个图片尺寸映射的新的图片上(map the coordinate of one point from a picture to a new picture of different size)
        :param point: 点的坐标(coordinate of point)
        :param now: 现在图片的尺寸(size of current picture)
        :param new: 新的图片尺寸(new picture size)
        :return: 新的点坐标(new point coordinate)
        """
        x, y = point
        now_w, now_h = now
        new_w, new_h = new
        new_x = x * new_w / now_w
        new_y = y * new_h / now_h
        
        return data_type(new_x), data_type(new_y)

    def get_area_max_contour(self, contours, min_area=50, max_area=640*480):
        """
        获取轮廓中面积最重大的一个, 过滤掉面积过小的情况(get the contour whose area is the largest. Filter out those whose area is too small)
        :param contours: 轮廓列表(contour list)
        :param threshold: 面积阈值, 小于这个面积的轮廓会被过滤(area threshold. Contour whose area is less than this value will be filtered out)
        :return: 如果最大的轮廓面积大于阈值则返回最大的轮廓, 否则返回None(if the maximum contour area is greater than this threshold, return the
        largest contour, otherwise return None)
        """
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # 历遍所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if max_area >= contour_area_temp >= min_area:  # 过滤干扰
                    area_max_contour = c

        return area_max_contour,  contour_area_max  # 返回最大的轮廓

    def detect(self, bgr_image):
        '''
        颜色检测
        :param image: 要进行颜色检测的原图像，格式为bgr，即opencv格式
        :return: 返回原图像和检测的物体的信息
        '''
        try:
            img_h, img_w = bgr_image.shape[:2]  # 获取原图大小
            image_draw = bgr_image.copy() 
            
            image_gb = cv2.GaussianBlur(bgr_image, (3, 3), 3)  # 高斯模糊去噪点
            image_resize = cv2.resize(image_gb, tuple(self.image_proc_size), interpolation=cv2.INTER_NEAREST)  # 图像缩放, 加快图像处理速度, 不能太小，否则会丢失大量细节

            image_lab = cv2.cvtColor(image_resize, cv2.COLOR_BGR2LAB)  # bgr空间转lab空间，方便提取颜色
            # 物体信息列表:颜色, 位置, 大小, 角度
            object_info_list = []
            center_x = 0
            center_y = 0
            for color in self.detect_info:  # 遍历颜色列表
                if color.use_name:
                    if color.color_name in self.lab_data:  # 如果要识别的颜色在lab里有
                        lower = tuple(self.lab_data[color.color_name]['min'])
                        upper = tuple(self.lab_data[color.color_name]['max'])
                    else:
                        continue
                else:
                    lower = tuple(color.lab_min)
                    upper = tuple(color.lab_max)

                if color.detect_type == 'line':  # 巡线检测
                    line_roi = [color.line_roi.up, color.line_roi.center, color.line_roi.down]
                    for roi in line_roi:
                        blob = image_lab[roi.y_min:roi.y_max, roi.x_min:roi.x_max]  # 截取roi
                        binary = cv2.inRange(blob, lower, upper)  # 二值化
                        dilated = self.erode_and_dilate(binary)  # 腐蚀膨胀
                        # cv2.imshow('1', cv2.cvtColor(dilated, cv2.COLOR_GRAY2RGB))
                        # continue
                        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出所有轮廓
                        max_contour, max_area = self.get_area_max_contour(contours, color.min_area, color.max_area)
                        if max_contour is not None:
                            rect = cv2.minAreaRect(max_contour)  # 获取最小外接矩形
                            corners = np.int64(cv2.boxPoints(rect))  # 获取最小外接矩形的四个角点
                            for j in range(4):
                                corners[j, 0], corners[j, 1] = self.point_remapped([corners[j, 0] + roi.x_min, corners[j, 1] + roi.y_min], self.image_proc_size, [img_w, img_h], data_type=int)  # 点映射到原图大小
                            x, y = self.point_remapped([rect[0][0] + roi.x_min, rect[0][1] + roi.y_min], self.image_proc_size, [img_w, img_h], data_type=int)  # 点映射到原图大小
                            
                            cv2.drawContours(image_draw, [corners], -1, (0, 255, 255), 2, cv2.LINE_AA)  # 绘制矩形轮廓
                            cv2.circle(image_draw, (x, y), 5, (0, 255, 255), -1)
                            center_x = x
                            center_y = y
                            break
# 颜色, 位置, 大小, 角度
                    if center_x != 0:
                        # print(center_x, center_y)
                        object_info_list.extend([[color.color_name, [center_x, center_y], [img_w, img_h], 0, 0]])
                        cv2.circle(image_draw, (center_x, center_y), 5, (0, 255, 255), -1)  # 画出中心点
                else:
                    roi = color.roi
                    if roi.y_min == roi.y_max:
                        blob = image_lab
                    else:
                        blob = image_lab[roi.y_min:roi.y_max, roi.x_min:roi.x_max]
                    
                    binary = cv2.inRange(image_lab, lower, upper)  # 二值化
                    dilated = self.erode_and_dilate(binary)  # 腐蚀膨胀
                    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出所有轮廓
                    max_contour, max_area = self.get_area_max_contour(contours, color.min_area, color.max_area)
                    if max_contour is not None:
                        if color.detect_type == 'rect':
                            rect = cv2.minAreaRect(max_contour)  # 获取最小外接矩形
                            corners = np.int64(cv2.boxPoints(rect))  # 获取最小外接矩形的四个角点
                            for j in range(4):
                                corners[j, 0], corners[j, 1] = self.point_remapped([corners[j, 0] + roi.x_min, corners[j, 1] + roi.y_min], self.image_proc_size, [img_w, img_h], data_type=int)  # 点映射到原图大小
                            x, y = self.point_remapped([rect[0][0] + roi.x_min, rect[0][1] + roi.y_min], self.image_proc_size, [img_w, img_h], data_type=int)  # 点映射到原图大小
                            
                            cv2.drawContours(image_draw, [corners], -1, (0, 255, 255), 2, cv2.LINE_AA)  # 绘制矩形轮廓
                            object_info_list.extend([[color.color_name, [x, y], [img_w, img_h], 0, int(rect[2])]])
                        else:
                            ((x, y), radius) = cv2.minEnclosingCircle(max_contour)  # 获取最小外接圆
                            x, y = self.point_remapped([x + roi.x_min, y + roi.y_min], self.image_proc_size, [img_w, img_h], data_type=int)  # 点映射到原图大小

                            radius = self.point_remapped([radius + roi.x_min, 0 + roi.y_min], self.image_proc_size, [img_w, img_h], data_type=int)[0]
                            cv2.circle(image_draw, (x, y), radius, (0, 255, 255), 2)  # 画圆
                            
                            object_info_list.extend([[color.color_name, [x, y], [img_w, img_h], radius, 0]])
            
            return image_draw, object_info_list  # 返回原图像和物体的信息
        except BaseException as e:
            print('color detect error:', e)
            return image_draw, [] 

if __name__ == '__main__':
    import time
    from ainex_sdk import common
    
    class ROI:
        y_min = 0
        y_max = 120
        x_min = 0
        x_max = 160
    
    class LineROI:
        up = ROI()
        center = ROI()
        down = ROI()
    
    class ColorDetect:
        color_name = ''
        use_name = True
        detect_type = 'circle'
        roi = ROI()
        line_roi = LineROI()
        min_area = 10
        max_area = 160*120
        def __init__(self, color_name):
            self.color_name = color_name
    
    class ColorsDetect:
        data = [ColorDetect('red'), ColorDetect('green'), ColorDetect('blue')]
    
    roi = [ # [ROI, weight]
        (20, 30, 40, 120), 
        (40, 50, 40, 120), 
        (60, 70, 40, 120)  
        ]
    line = ColorDetect('black')
    line.detect_type = 'line'
    line.line_roi.up.y_min = roi[0][0]
    line.line_roi.up.y_max = roi[0][1]
    line.line_roi.up.x_min = roi[0][2]
    line.line_roi.up.x_max = roi[0][3]

    line.line_roi.center.y_min = roi[1][0]
    line.line_roi.center.y_max = roi[1][1]
    line.line_roi.center.x_min = roi[1][2]
    line.line_roi.center.x_max = roi[1][3]

    line.line_roi.down.y_min = roi[2][0]
    line.line_roi.down.y_max = roi[2][1]
    line.line_roi.down.x_min = roi[2][2]
    line.line_roi.down.x_max = roi[2][3]
    data = ColorsDetect()
    data.data = [line]

    lab_config = common.get_yaml_data('/home/ubuntu/software/lab_tool/lab_config.yaml')
    # color_detection = ColorDetection(lab_config['lab']['Mono'], data, [160, 120])
    color_detection = ColorDetection(lab_config['lab']['Mono'], ColorsDetect(), [160, 120])
    cap = cv2.VideoCapture('/dev/video0')
    while True:
        ret, frame = cap.read()
        if ret:
            img = color_detection.detect(frame)[0]
            cv2.imshow('img', img)
            key = cv2.waitKey(1)
            if key != -1:
                break
        else:
            time.sleep(0.01)
    cap.release()
