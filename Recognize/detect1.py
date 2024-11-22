import cv2

def detect(readimg):
    # Canny 边缘检测
    canny = cv2.Canny(readimg, 80, 300)

    # 圆形检测
    circles = cv2.HoughCircles(
        canny, cv2.HOUGH_GRADIENT, 1, 30, param1=300, param2=30, minRadius=30, maxRadius=200)

    if circles is None:
        return 0, 0, 0
    else:
        return circles[0, 0, 2], circles[0, 0, 0], circles[0, 0, 1]