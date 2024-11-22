import cv2


def detect(readimg):
    # canny 边缘检测
    canny = cv2.Canny(readimg, 40, 80)

    # 圆形检测
    circles = cv2.HoughCircles(
        canny, cv2.HOUGH_GRADIENT, 1, 30, param1=80, param2=30, minRadius=7, maxRadius=30)

    if circles is None:
        return 0, 0, 0
    else:
        return circles[0, 0, 2], circles[0, 0, 0], circles[0, 0, 1]
