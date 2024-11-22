import cv2
import numpy as np
import os
import pandas as pd
import csv
import timeit

import os
from IPython.display import Image, display

def getpos(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(img[y,x])

def detect(img):
    # 转换 HSV 色彩空间
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 通过 HSV 对应的绿色色彩区间对绿色部分进行提取
    lower = np.array([60, 43, 46])
    upper = np.array([100, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)

    # 图像腐蚀 去除一些干扰小区域 只保留最大的绿色板
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel)

    # 获取坐标和长度
    index_x = np.where(mask.sum(0) > 0)[0]
    index_y = np.where(mask.sum(1) > 0)[0]
    offset = 5
    if len(index_x) and len(index_y):
        x0 = index_x[0]
        x1 = index_x[-1]
        y1 = index_y[-1] - offset
        return x1-x0, x0, y1
    else:
        return 0, 0, 0


img_name = '05.jpg'

# 获取路径
img_path = os.path.join('images', img_name)
save_path = os.path.join('outputs', img_name)

# 读取图像
img = cv2.imread(img_path)

# 图像检测
r, x, y = (int(x) for x in detect(img))

# 绘制结果
cv2.circle(img, (x, y), 3, (225, 0, 0), 5)
cv2.circle(img, (x, y), r, (225, 0, 0), 3)

# 保存并显示
cv2.imwrite(save_path, img)
display(Image(save_path))

# 显示结果图片
cv2.imshow("image", img)
cv2.setMouseCallback('image',getpos)
cv2.waitKey()
cv2.destroyAllWindows()

print(r, x, y)