#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, rospy, time, os, math
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
import matplotlib.pyplot as plt
from control import cone_detector,lane_follower, traffic_light, obstacle_detector

#=============================================
# image -> 카메라 이미지를 담을 변수
# ranges -> 라이다 데이터를 담을 변수
# motor -> 모터노드
# motor_msg -> 모터 토픽 메시지
# bridge -> OpenCV 함수를 사용하기 위한 브릿지 
#=============================================
image = np.empty(shape=[0])
ranges = None
motor = None
motor_msg = XycarMotor()
bridge = CvBridge()

def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def lidar_callback(data):
    global ranges    
    ranges = data.ranges[0:360]
	
def drive(angle, speed):
    motor_msg.angle = float(angle)
    motor_msg.speed = float(speed)
    motor.publish(motor_msg)
             
def start():
    global motor, image, ranges
    
    print("Start program --------------")
    rospy.init_node('Track_Driver')
    rospy.Subscriber("/usb_cam/image_raw/",Image,usbcam_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)
    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")
    rospy.wait_for_message("/scan", LaserScan)
    print("Lidar Ready ----------")

    #=========================================
    # 먼저 신호등의 초록불을 탐지하기 전까지 대기
    # 초록불을 탐지하면 차선 인식하며 주행
    # traffic cone 구간을 완료하지 않았고
    # traffic cone으로 인식되는 장애물을 만나면 traffic cone 회피모드로 주행
    # 이후 다시 차선 인식하며 주행
    # traffic cone 구간을 완료했고
    # 차량으로 인식되는 장애물을 만나면 차량 회피모드로 주행
    #     
    # ang -> 코드 내에서 수정 후 drive 함수의 인수로 넣어줄 조향각
    # spd -> 코드 내에서 수정 후 drive 함수의 인수로 넣어줄 속도
    # green_light -> 신호등의 초록불 탐지 여부
    # cone_start -> traffic cone 구간을 주행 시작했으면 True
    # cone_done -> traffic cone 구간을 주행 완료했으면 True
    # lane_change -> 차선을 변경해야할 때 True
    #=========================================

    ang = 0
    spd = 0
    green_light = False
    cone_start = False
    cone_done = False
    lane_change = False

    while not rospy.is_shutdown():

        if traffic_light(image) and not green_light:
            green_light = True

        elif green_light:
            spd, ang , lane_left = lane_follower(image,spd,ang,cone_start,cone_done,lane_change)

            if ranges is not None:
                dists = np.array([round(d,1) for d in ranges])

                if not cone_done:
                    spd, ang, cone_start,cone_done = cone_detector(dists,spd,ang)
                else:
                    spd, ang, lane_change = obstacle_detector(dists,spd,ang,lane_left)


        drive(angle=ang, speed=spd)
        time.sleep(0.1)
        
        cv2.waitKey(1)

if __name__ == '__main__':
    start()