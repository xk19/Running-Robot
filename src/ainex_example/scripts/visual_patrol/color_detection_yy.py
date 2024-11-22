#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/06/03
# @author:aiden
# 识别颜色以及计算物体的位置信息
import cv2
import math
import copy
import numpy as np


class ColorDetection_yy:
    def __init__(self, lab_config, detect_info, debug=True):
        '''
        lab_config: lab阈值，字典形式
        detect_info: 检测类型，ros格式
        '''
        self.debug = debug
        self.lab_data = lab_config
        self.detect_info = detect_info.data
        self.image_process_size = [160, 120]  # 图像处理大小

    def update_lab_config(self, lab_data):
        '''
        更新lab参数
        :param lab_data: lab阈值，字典形式
        :return:
        '''
        self.lab_data = lab_data

    def update_detect_info(self, detect_info):
        '''
        更新检测类型
        :param detect_info: 检测类型，ros格式
        '''
        self.detect_info = detect_info

    def value_remapped(self, x, in_min, in_max, out_min, out_max, data_type=float):
        '''
        将一个值从给定区间映射到另一个区间
        param x: 当前输入值
        param in_min: 当前区间最小值
        param in_max: 当前区间最大值
        param out_min: 映射到的区间最小值
        param out_max: 映射到的区间最大值
        param data_type: 最后的结构类型
        '''
        return data_type((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

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
        new_x = self.value_remapped(x, 0, now_w, 0, new_w)
        new_y = self.value_remapped(y, 0, now_h, 0, new_h)

        return data_type(new_x), data_type(new_y)

    def get_area_max_contour(self, contours, min_area=50, max_area=640 * 480):
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

        return area_max_contour, contour_area_max  # 返回最大的轮廓

    def get_lab_binary(self, lab_image, lower, upper, kernel=3):
        '''
        将lab二值化，并进行腐蚀膨胀
        param lab_image: 输入转换为lab后的图像
        param lower: 阈值的第一个区间
        param upper: 阈值的第二个区间
        param kernel: 腐蚀膨胀的核大小
        '''
        binary = cv2.inRange(lab_image, lower, upper)  # 二值化
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel, kernel))
        eroded = cv2.erode(binary, element)  # 腐蚀
        dilated = cv2.dilate(eroded, element)  # 膨胀

        return dilated

    def get_binary_contour(self, binary, min_area, max_area):
        contours = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出所有轮廓
        max_contour, contour_area = self.get_area_max_contour(contours, min_area, max_area)

        return max_contour, contour_area

    def get_lab_contour(self, lab_image, lower, upper, min_area, max_area):
        '''
        将lab转换为二值图并获取最大轮廓
        param lab_image: 输入转换为lab后的图像
        param lower: 阈值的第一个区间
        param upper: 阈值的第二个区间
        param min_area: 轮廓的最小面积
        param max_area: 轮廓的最大面积
        '''
        dilated = self.get_lab_binary(lab_image, lower, upper)
        max_contour, contour_area = self.get_binary_contour(dilated, min_area, max_area)

        return max_contour, contour_area

    def cal_line_angle(self, point1, point2):
        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]

        return int(math.degrees(math.atan2(dy, dx)))

    def cal_two_lines_angle(self, line1_point1, line1_point2, line2_point1, line2_point2):
        if line1_point2[0] == line1_point1[0]:
            slope1 = float('inf')
        else:
            # 计算直线A的斜率
            slope1 = (line1_point2[1] - line1_point1[1]) / (line1_point2[0] - line1_point1[0])

        if line2_point2[0] == line2_point1[0]:
            slope2 = float('inf')
        else:
            # 计算直线B的斜率
            slope2 = (line2_point2[1] - line2_point1[1]) / (line2_point2[0] - line2_point1[0])

        # 如果直线A和直线B的斜率相等，则夹角为0或180度，具体取决于它们的位置
        if slope1 == slope2:
            if line2_point1[0] < line1_point1[0] < line2_point2[0] or line2_point2[0] < line1_point1[0] < line2_point1[
                0]:
                angle = 0
            else:
                angle = 180
        else:
            # 否则，使用atan2函数计算夹角
            angle = math.degrees(math.atan2(slope2 - slope1, 1 + slope1 * slope2))

        return angle

    def find_intersection(self, k, x1, y1, x2, y2, x3, y3):
        """
        已知直线A的两个端点坐标(x1, y1)和(x2, y2)，以及斜率k，直线B过点(x3, y3)，斜率也是k
        从点(x1, y1)做一条垂线，与B相交于点(x4, y4)，从(x2, y2)做一条垂线，与B相交于点(x5, y5)
        """
        # 直线B的斜率和截距
        if k != 0:
            b_slope = k
            b_intercept = y3 - k * x3

            # 从点(x1, y1)做的垂线斜率和截距
            v1_slope = -1 / k
            v1_intercept = y1 - v1_slope * x1

            # 从点(x2, y2)做的垂线斜率和截距
            v2_slope = -1 / k
            v2_intercept = y2 - v2_slope * x2

            # 求交点
            x4 = (v1_intercept - b_intercept) / (b_slope - v1_slope)
            y4 = b_slope * x4 + b_intercept

            x5 = (v2_intercept - b_intercept) / (b_slope - v2_slope)
            y5 = b_slope * x5 + b_intercept
        else:
            x4, y4 = x1, y3
            x5, y5 = x2, y3

        return [int(x4), int(y4)], [int(x5), int(y5)]

    def detect_side(self, contour, roi, img_proc_w, img_proc_h, img_w, img_h):
        '''
        检测矩形最靠下的边
        param contour: 输入轮廓
        param roi: roi区域
        param img_w: 原图width
        param img_h: 原图height
        '''
        x, y, angle, corners = self.detect_rect(contour, roi, img_proc_w, img_proc_h, img_w, img_h)

        # 将矩形的四个角点作为拟合点
        pts = np.array([corners[0], corners[1], corners[2], corners[3]], np.int32)
        pts = pts.reshape((-1, 1, 2))
        # 使用cv2.fitLine()函数将矩形拟合成一条线
        [cosx, sinx, x0, y0] = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)

        k = math.atan2(sinx, cosx)
        # y = k(x - x0) + y0

        # 计算线段的两个端点
        min_left_x = (contour[contour[:, :, 0].argmin()][0])[0]
        max_right_x = (contour[contour[:, :, 0].argmax()][0])[0]

        min_left_x = self.value_remapped(min_left_x + roi.x_min, 0, img_proc_w, 0, img_w)
        max_right_x = self.value_remapped(max_right_x + roi.x_min, 0, img_proc_w, 0, img_w)

        min_left_y = int(k * (min_left_x - x0) + y0)
        max_right_y = int(k * (max_right_x - x0) + y0)

        # 计算矩形最靠下的边的端点
        max_down_x, max_down_y = contour[contour[:, :, 1].argmax()][0]
        max_down_x, max_down_y = self.point_remapped([max_down_x + roi.x_min, max_down_y + roi.y_min],
                                                     [img_proc_w, img_proc_h], [img_w, img_h], data_type=int)

        [x3, y3], [x4, y4] = self.find_intersection(k, min_left_x, min_left_y, max_right_x, max_right_y, max_down_x,
                                                    max_down_y)

        return x3, y3, x4, y4, int(math.degrees(math.atan(k)))

    def detect_cross(self, binary, contour, roi, min_area, max_area, img_proc_w, img_proc_h, img_w, img_h,
                     angle_threshold=20):
        '''
        检测十字路口
        '''
        # 获取轮廓最左，最右，最上的点
        min_left_x, min_left_y = contour[contour[:, :, 0].argmin()][0]
        max_right_x, max_right_y = contour[contour[:, :, 0].argmax()][0]
        min_up_x, min_up_y = contour[contour[:, :, 1].argmin()][0]

        # 截取离左右点2/3的区域
        center_x, center_y = int((min_left_x + max_right_x) / 2), int((min_left_y + max_right_y) / 2)

        half_center_x1, half_center_y1 = int(min_left_x + 2 * (center_x - min_left_x) / 3), int(
            (min_left_y + center_y) / 2)

        half_center_x2, half_center_y2 = int(center_x + (max_right_x - center_x) / 3), int((max_right_y + center_y) / 2)

        left_y1 = min(min_left_y, half_center_y1) - 10
        right_y1 = max(min_left_y, half_center_y1) + 10
        left_y2 = min(max_right_y, half_center_y2) - 10
        right_y2 = max(max_right_y, half_center_y2) + 10
        if left_y1 < 0:
            left_y1 = 0
        if right_y1 > img_proc_h:
            right_y1 = img_proc_h
        if left_y2 < 0:
            left_y2 = 0
        if right_y2 > img_proc_h:
            right_y2 = img_proc_h

        roi1 = binary[left_y1:right_y1, min(min_left_x, half_center_x1):max(min_left_x, half_center_x1)]
        roi2 = binary[left_y2:right_y2, min(max_right_x, half_center_x2):max(max_right_x, half_center_x2)]

        max_contour1, contour_area1 = self.get_binary_contour(roi1, min_area, max_area)

        max_contour2, contour_area2 = self.get_binary_contour(roi2, min_area, max_area)

        if max_contour1 is not None and max_contour2 is not None:
            # 识别最靠下的线
            roi_1 = copy.deepcopy(roi)
            roi_1.y_min = left_y1 + roi.y_min
            roi_1.y_max = right_y1 + roi.y_min
            roi_1.x_min = min(min_left_x, half_center_x1) + roi.x_min
            roi_1.x_max = max(min_left_x, half_center_x1) + roi.x_min

            x1, y1 = self.detect_side(max_contour1, roi_1, img_proc_w, img_proc_h, img_w, img_h)[:2]

            roi_2 = copy.deepcopy(roi)
            roi_2.y_min = left_y2 + roi.y_min
            roi_2.y_max = right_y2 + roi.y_min
            roi_2.x_min = min(max_right_x, half_center_x2) + roi.x_min
            roi_2.x_max = max(max_right_x, half_center_x2) + roi.x_min
            x2, y2 = self.detect_side(max_contour2, roi_2, img_proc_w, img_proc_h, img_w, img_h)[2:4]

            # 计算横竖线的角度来判断是否是十字
            angle = self.cal_two_lines_angle([x1, y1], [x2, y2], [min_up_x, min_up_y], [center_x, center_y])

            if abs(abs(angle) - 89) < angle_threshold:
                return x1, y1, x2, y2, self.cal_line_angle([x1, y1], [x2, y2])
            else:
                return False
        else:
            return False

    def detect_rect(self, contour, roi, img_proc_w, img_proc_h, img_w, img_h):
        '''
        获取轮廓的最小外接矩形的中心和角点
        param contour: 输入轮廓
        param roi: roi区域
        param img_w: 原图width
        param img_h: 原图height
        '''
        rect = cv2.minAreaRect(contour)  # 获取最小外接矩形
        corners = np.int64(cv2.boxPoints(rect))  # 获取最小外接矩形的四个角点
        for j in range(4):
            corners[j, 0], corners[j, 1] = self.point_remapped([corners[j, 0] + roi.x_min, corners[j, 1] + roi.y_min],
                                                               [img_proc_w, img_proc_h], [img_w, img_h],
                                                               data_type=int)  # 点映射到原图大小
        x, y = self.point_remapped([rect[0][0] + roi.x_min, rect[0][1] + roi.y_min], [img_proc_w, img_proc_h],
                                   [img_w, img_h], data_type=int)  # 点映射到原图大小
        return x, y, int(rect[2]), corners

    def detect_circle(self, contour, roi, img_proc_w, img_proc_h, img_w, img_h):
        '''
        获取最小外接圆的中心和半径
        param contour: 输入轮廓
        param roi: roi区域
        param img_w: 原图width
        param img_h: 原图height
        '''
        ((x, y), radius) = cv2.minEnclosingCircle(contour)  # 获取最小外接圆
        x, y = self.point_remapped([x + roi.x_min, y + roi.y_min], [img_proc_w, img_proc_h], [img_w, img_h],
                                   data_type=int)  # 点映射到原图大小
        radius = self.point_remapped([radius + roi.x_min, 0 + roi.y_min], [img_proc_w, img_proc_h], [img_w, img_h],
                                     data_type=int)[0]

        return x, y, radius

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

            # 物体信息列表:颜色, 位置, 大小, 角度
            object_info_list = []
            center_x = 0
            center_y = 0
            last_process_size = [0, 0]
            for color_info in self.detect_info:  # 遍历颜色列表
                self.image_process_size = color_info.image_process_size
                if self.image_process_size != last_process_size:
                    image_resize = cv2.resize(image_gb, tuple(self.image_process_size),
                                              interpolation=cv2.INTER_NEAREST)  # 图像缩放, 加快图像处理速度, 不能太小，否则会丢失大量细节
                    image_lab = cv2.cvtColor(image_resize, cv2.COLOR_BGR2LAB)  # bgr空间转lab空间，方便提取颜色

                last_process_size = self.image_process_size
                if color_info.use_name:
                    if color_info.color_name in self.lab_data:  # 如果要识别的颜色在lab里有
                        lower = tuple(self.lab_data[color_info.color_name]['min'])
                        upper = tuple(self.lab_data[color_info.color_name]['max'])
                    else:
                        continue
                else:
                    lower = tuple(color_info.lab_min)
                    upper = tuple(color_info.lab_max)

                if color_info.detect_type == 'line':  # 巡线检测
                    line_roi = [color_info.line_roi.up, color_info.line_roi.center, color_info.line_roi.down]
                    for roi in line_roi:
                        blob = image_lab[roi.y_min:roi.y_max, roi.x_min:roi.x_max]  # 截取roi
                        max_contour, contour_area = self.get_lab_contour(blob, lower, upper, color_info.min_area,
                                                                         color_info.max_area)
                        if max_contour is not None:
                            x, y, angle, corners = self.detect_rect(max_contour, roi, self.image_process_size[0],
                                                                    self.image_process_size[1], img_w, img_h)
                            cv2.circle(image_draw, (x, y), 5, (0, 255, 255), -1)
                            cv2.drawContours(image_draw, [corners], -1, (0, 255, 255), 2, cv2.LINE_AA)  # 绘制矩形轮廓
                            center_x = x
                            center_y = y
                            break

                    # 颜色, 位置, 大小, 角度
                    if center_x != 0:
                        object_info_list.extend([[color_info.color_name, color_info.detect_type, [center_x, center_y],
                                                  [img_w, img_h], 0, 0, [], []]])
                        cv2.circle(image_draw, (center_x, center_y), 5, (0, 255, 255), -1)  # 画出中心点
                else:
                    roi = color_info.roi
                    if roi.y_min == roi.y_max:
                        blob = image_lab
                    else:
                        blob = image_lab[roi.y_min:roi.y_max, roi.x_min:roi.x_max]
                    if self.debug:
                        x_min, y_min = self.point_remapped([roi.x_min, roi.y_min], self.image_process_size,
                                                           [img_w, img_h], data_type=int)
                        x_max, y_max = self.point_remapped([roi.x_max, roi.y_max], self.image_process_size,
                                                           [img_w, img_h], data_type=int)
                        cv2.rectangle(image_draw, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                    if color_info.detect_type != 'cross':
                        max_contour, contour_area = self.get_lab_contour(blob, lower, upper, color_info.min_area,
                                                                         color_info.max_area)
                        if max_contour is not None:
                            if color_info.detect_type == 'rect':
                                x, y, angle, corners = self.detect_rect(max_contour, roi, self.image_process_size[0],
                                                                        self.image_process_size[1], img_w, img_h)
                                cv2.drawContours(image_draw, [corners], -1, (0, 255, 255), 2, cv2.LINE_AA)  # 绘制矩形轮廓
                                object_info_list.extend([[color_info.color_name, color_info.detect_type, [x, y],
                                                          [img_w, img_h], 0, angle, [], []]])
                            elif color_info.detect_type == 'side':
                                x1, y1, x2, y2, angle = self.detect_side(max_contour, roi, self.image_process_size[0],
                                                                         self.image_process_size[1], img_w, img_h)
                                cv2.line(image_draw, (x1, y1), (x2, y2), (0, 255, 255), 3)
                                x, y = int((x1 + x2) / 2), int((y1 + y2) / 2)
                                object_info_list.extend([[color_info.color_name, color_info.detect_type, [x, y],
                                                          [img_w, img_h], 0, angle, [x1, y1], [x2, y2]]])
                            elif color_info.detect_type == 'circle':
                                x, y, radius = self.detect_circle(max_contour, roi, self.image_process_size[0],
                                                                  self.image_process_size[1], img_w, img_h)
                                cv2.circle(image_draw, (x, y), radius, (0, 255, 255), 2)  # 画圆
                                object_info_list.extend([[color_info.color_name, color_info.detect_type, [x, y],
                                                          [img_w, img_h], radius, 0, [], []]])
                    else:
                        binary = self.get_lab_binary(blob, lower, upper)
                        max_contour, contour_area = self.get_binary_contour(binary, color_info.min_area,
                                                                            color_info.max_area)
                        if max_contour is not None:
                            result = self.detect_cross(binary, max_contour, roi, color_info.min_area,
                                                       color_info.max_area, self.image_process_size[0],
                                                       self.image_process_size[1], img_w, img_h)
                            if result:
                                x1, y1, x2, y2, angle = result
                                cv2.line(image_draw, (x1, y1), (x2, y2), (255, 0, 0), 3)
                                x, y = int((x1 + x2) / 2), int((y1 + y2) / 2)
                                object_info_list.extend([[color_info.color_name, color_info.detect_type, [x, y],
                                                          [img_w, img_h], 0, angle, [x1, y1], [x2, y2]]])

            return image_draw, object_info_list  # 返回原图像和物体的信息
        except BaseException as e:
            print('color detect error:', e)
            return image_draw, []


if __name__ == '__main__':
    import time
    from ainex_sdk import common

    print('hello yy')
    while True:
        class ROI:
            y_min = 30
            y_max = 90
            x_min = 0
            x_max = 160


        class LineROI:
            up = ROI()
            center = ROI()
            down = ROI()


        class ColorDetect:
            color_name = ''
            use_name = True
            detect_type = 'rect'
            image_process_size = [160, 120]
            roi = ROI()
            line_roi = LineROI()
            min_area = 10
            max_area = 160 * 120

            def __init__(self, color_name):
                self.color_name = color_name


        class ColorsDetect:
            data = [ColorDetect('sign')]


        lab_config = common.get_yaml_data('/home/ubuntu/software/lab_tool/lab_config.yaml')
        # color_detection = ColorDetection_yy(lab_config['lab']['Mono'], data)
        color_detection = ColorDetection_yy(lab_config['lab']['Mono'], ColorsDetect())
        cap = cv2.VideoCapture('/dev/video0')
        flag = 0
        while True:
            ret, frame = cap.read()
            if ret:
                img = color_detection.detect(frame)[0]
                cv2.imshow('img', img)
                if len(color_detection.detect(frame)[1]):
                    print('green')
                    flag = 1
                    break
                key = cv2.waitKey(1)
                if key != -1:
                    break
            else:
                time.sleep(0.01)
        cap.release()
        if flag == 1:
            flag = 0
            break

    print('hello kk')