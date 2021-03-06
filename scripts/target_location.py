#!/usr/bin/python
#-*- encoding: utf8 -*-
# 简单的基于OpenCV的目标识别（识别、定位红色小球）

import rospy
import cv2
import numpy as np
import math
from uav_sim.msg import PointWithState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class TargetLocationNode:
    
    def __init__(self):
        rospy.init_node('target_location_node', anonymous=True)
        self.orgFrame_ = None
        self.bridge_ = CvBridge()
        self.location_pub_ = rospy.Publisher("/target_location/location", PointWithState, queue_size=10)
        self.image_result_pub_ = rospy.Publisher("/target_location/image_result", Image, queue_size=10)
        self.image_sub_ = rospy.Subscriber("/iris/usb_cam/image_raw", Image, self.imagesubCallback)
        self.location_timer_ = rospy.Timer(rospy.Duration(0.05), self.locationloopCallback)

        #以下变量仅用于本场景中的目标（红色小球）识别与定位
        self.color_range_ = [(0,43,36), (6, 255, 255)]
        
        self.initFromYaml()

        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
        cv2.destroyAllWindows()


    def initFromYaml(self):
        self.f_ = rospy.get_param('/target_location/focal_length')  #相机内参
        self.r_ = rospy.get_param('/target_location/target_radius')  #目标小球的真实半径（米）


    def imagesubCallback(self, data):
        try:
            self.orgFrame_ = self.bridge_.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as err:
            print(err)


    def locationloopCallback(self, event):
        if self.orgFrame_ is not None:
            orgFrame_copy = self.orgFrame_.copy()
            height = orgFrame_copy.shape[0]
            width = orgFrame_copy.shape[1]

            frame = cv2.resize(self.orgFrame_, (width, height), interpolation = cv2.INTER_CUBIC) #将图片缩放     
            frame = cv2.GaussianBlur(frame, (3,3), 0)  #高斯模糊
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  #将图片转换到HSV空间
            h, s, v = cv2.split(frame)  #分离出各个HSV通道
            v = cv2.equalizeHist(v)  #直方图化
            frame = cv2.merge((h,s,v))  #合并三个通道

            rad = 0
            areaMaxContour = 0
            area_max = 0
            centerX = 0
            centerY = 0
            isTargetFound = False
        
            frame = cv2.inRange(frame, self.color_range_[0], self.color_range_[1])  #对原图像和掩模进行位运算
            opened = cv2.morphologyEx(frame, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))  #开运算
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3,3),np.uint8))  #闭运算
            (image, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  #找出轮廓

            #在contours中找出最大轮廓
            contour_area_temp = 0
            contour_area_max = 0
            area_max_contour = None
            for c in contours : #遍历所有轮廓
                contour_area_temp = math.fabs(cv2.contourArea(c)) #计算轮廓面积
                if contour_area_temp > contour_area_max :
                    contour_area_max = contour_area_temp
                    area_max_contour = c
 
            if area_max_contour is not None:
                if contour_area_max > 50:
                    isTargetFound = True

            if isTargetFound:
                target = 'Red'
                ((centerX, centerY), rad) = cv2.minEnclosingCircle(area_max_contour)  #获取最小外接圆
                cv2.circle(orgFrame_copy, (int(centerX), int(centerY)), int(rad), (0, 255, 0), 2)#画出圆心   
            else:
                target = 'None'
                pass
                
            #计算球心在相机坐标系下的坐标
            if target == 'Red':
                Z = math.sqrt((self.f_*self.r_/rad)**2 + self.r_**2)
                X = (centerX-width/2)*Z/self.f_
                Y = (centerY-height/2)*Z/self.f_
            else:
                pass

            location = PointWithState()
            if target == 'Red':
                location.state = 1
                location.position.x = Z #针孔成像使用的坐标系与PX4中的坐标系需要转换
                location.position.y = -X
                location.position.z = -Y
            else:
                location.state = 0
                location.position.x = 0
                location.position.y = 0
                location.position.z = 0
            self.location_pub_.publish(location)
            self.image_result_pub_.publish(self.bridge_.cv2_to_imgmsg(orgFrame_copy))


if __name__ == '__main__':
    tl = TargetLocationNode()
