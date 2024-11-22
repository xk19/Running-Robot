import cv2
import os
import numpy as np

from IPython.display import Image, display

def getpos(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(img[y,x])

img_name = '02.jpg'

# 获取路径
img_path = os.path.join('images', img_name)
save_path = os.path.join('outputs', img_name)

# 读取图像
img = cv2.imread(img_path)

# Canny 边缘检测
canny = cv2.Canny(img, 80, 300)

# 圆形检测
circles = cv2.HoughCircles(
    canny, cv2.HOUGH_GRADIENT, 1, 30, param1=300, param2=30, minRadius=30, maxRadius=200)

if circles is None:
    r, x, y = (0, 0, 0)
else:
    r, x, y = (int(circles[0, 0, 2]), int(circles[0, 0, 0]), int(circles[0, 0, 1]))

print(r, x, y)

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

