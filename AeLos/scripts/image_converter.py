#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys

sys.path.append("/home/lemon/catkin_ws/src/aelos_smart_ros")
from leju import base_action


class ImgConverter():
    def __init__(self):
        self.bridge = CvBridge()        
        self.sub_chest = rospy.Subscriber('/usb_cam_chest/image_raw', Image, self.cb_chest)
        self.sub_head = rospy.Subscriber('/usb_cam_head/image_raw', Image, self.cb_head)
        self.img_chest = None
        self.img_head = None
        

    def cb_chest(self, msg):
        cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.img_chest = cv2_img
        

    def cb_head(self, msg):
        cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.img_head = cv2_img

    def chest_image(self):
        return True, self.img_chest

    def head_image(self):
        return True, self.img_head
        

def main():
    try:
        rospy.init_node('image_listener')
        print('Node init')
        image_reader = ImgConverter()
        
        while True:
            rospy.spin()
            time.sleep(0.01)
            
    except rospy.ROSInterruptException:
        pass
    
# testing
if __name__ == '__main__':
    main()
