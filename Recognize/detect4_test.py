import cv2
import numpy as np
import os
from IPython.display import Image, display


def detect(img):
    # 转换 HSV 色彩空间
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_copy = hsv[150:340][:][:]

    # 通过 HSV 对应的绿色色彩区间对绿色部分进行提取
    lower = np.array([105, 43, 46])
    upper = np.array([120, 255, 255])
    mask = cv2.inRange(hsv_copy, lower, upper)

    # 图像腐蚀 去除一些干扰小区域 只保留最大的绿色板
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel)

    # 获取坐标和长度
    index_x = np.where(mask.sum(0) > 0)[0]
    index_y = np.where(mask.sum(1) > 0)[0]
    # offset = 5
    if len(index_x) and len(index_y):
        x0 = index_x[0] + 25
        x1 = index_x[-1] - 15
        y1 = index_y[-1] - 15
        return x1 - x0, x0, y1 + 150
    else:
        return 0, 0, 0