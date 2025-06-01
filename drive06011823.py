#!/usr/bin/env python
# -*- coding: utf-8 -*- 2
#=============================================
# 본 프로그램은 2025 제8회 국민대 자율주행 경진대회에서
# 예선과제를 수행하기 위한 파일입니다. 
# 예선과제 수행 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, rospy, time, os, math
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
import matplotlib.pyplot as plt
from collections import deque

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
ranges = None  # 라이다 데이터를 담을 변수
motor = None  # 모터노드
motor_msg = XycarMotor()  # 모터 토픽 메시지
bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지 

#=============================================
# 콜백함수 - 카메라 토픽을 처리하는 콜백함수
#=============================================
def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")
   
#=============================================
# 콜백함수 - 라이다 토픽을 받아서 처리하는 콜백함수
#=============================================
def lidar_callback(data):
    global ranges    
    ranges = data.ranges[0:360]
	
#=============================================
# 모터로 토픽을 발행하는 함수 
#=============================================
def drive(angle, speed):
    motor_msg.angle = float(angle)
    motor_msg.speed = float(speed)
    motor.publish(motor_msg)
             
#=============================================
# 실질적인 메인 함수 
#=============================================
def start():

    global motor, image, ranges
    
    print("Start program --------------")

    #=========================================
    # 노드를 생성하고, 구독/발행할 토픽들을 선언합니다.
    #=========================================
    rospy.init_node('Track_Driver')
    rospy.Subscriber("/usb_cam/image_raw/",Image,usbcam_callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)
        
    #=========================================
    # 노드들로부터 첫번째 토픽들이 도착할 때까지 기다립니다.
    #=========================================
    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")
    rospy.wait_for_message("/scan", LaserScan)
    print("Lidar Ready ----------")

    #=========================================
    # 메인 루프 
    #=========================================

    ang = 0
    spd = 0
    basic_spd = 25
    cone_cnt = 0
    angle_increment = 12
    min_left, max_left = 10, 55
    min_right, max_right = 305, 350
    min_lr_dist = 4
    min_front_dist = 8
    green_light = False
    lane_left = True
    lane_change = False
    cone_start = False
    cone_done = False
    avg_err = deque(maxlen=10)

    #=========================================
    # ang -> 핸들 조향각(실제로 넘겨줄 값)
    # angle_increment -> 조향각 조정 시, 각 if 블록마다 증가하는 각도
    # min_left, max_left -> 좌측 장애물 탐지각(최소, 최대)
    # min_right, max_right -> 우측 장애물 탐지각(최소, 최대)
    # min_lr_dist -> 좌, 우측 장애물 탐지 최소 거리
    #
    # lane_left -> 차선이 왼쪽이면 True
    # cone_done -> traffic cone 회피를 끝냈으면 True
    #=========================================
    
    while not rospy.is_shutdown():
        height, width, _ = image.shape
        
        # ROI 설정 (하단 40%만 사용)
        roi = image[int(height * 0.6):, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        if not green_light:
            green_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lower_green = (45, 100, 100)
            upper_green = (85, 255, 255)
            green_mask = cv2.inRange(green_hsv, lower_green, upper_green)

            if cv2.countNonZero(green_mask) > 500:
                green_light = True
                rospy.loginfo("출발 신호 감지 - 출발!")

        else:
            # 노란 점선 마스크
            yellow_lower = np.array([20, 100, 100])
            yellow_upper = np.array([30, 255, 255])
            yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

            # 흰색 실선 마스크
            white_lower = np.array([0, 0, 200])
            white_upper = np.array([180, 30, 255])
            white_mask = cv2.inRange(hsv, white_lower, white_upper)

            center_image = width // 2
            spd = basic_spd

            M_yellow = cv2.moments(yellow_mask)

            if lane_change:
                if lane_before != lane_left:
                    print('change success')
                    lane_change = False
                
                if lane_change:
                    lane_before = lane_left
                    spd *= 2

                    if lane_left:
                        ang = 30
                    else: 
                        ang = -30

            if M_yellow['m00'] > 0 and (not cone_start or cone_done):
                cx_yellow = int(M_yellow['m10'] / M_yellow['m00'])

                # 노란 점선이 화면의 오른쪽 → 왼쪽 흰선 사용
                if cx_yellow > center_image:
                    selected_white_mask = white_mask[:, :center_image]
                    offset = 0

                    lane_left = True
                # 노란 점선이 왼쪽 → 오른쪽 흰선 사용
                else:
                    selected_white_mask = white_mask[:, center_image:]
                    offset = center_image

                    lane_left = False

                M_white = cv2.moments(selected_white_mask)

                if M_white['m00'] > 0 and not lane_change:
                    cx_white = int(M_white['m10'] / M_white['m00']) + offset

                    # 노란 점선과 해당 흰선 사이의 중앙 계산
                    center_x = (cx_yellow + cx_white) // 2
                    error = center_x - center_image
                    print(error)
                    
                    if not lane_change:
                        if abs(error) > 50:
                            if len(avg_err) > 3:
                                avg_err.clear()
                        else: 
                            avg_err.append(abs(error))
                        
                        if len(avg_err) > 7:
                            print(error, np.mean(avg_err))
                            if np.mean(avg_err) < 20:
                                spd *= 2.5
                            elif np.mean(avg_err) < 35:
                                spd *= 2
                            elif np.mean(avg_err) < 50:
                                spd *= 1.5
                    # 조향 명령 설정
                    ang = (float(error) / 100) * 55

            if ranges is not None:
                dists = np.array([round(d,1) for d in ranges])
                if not cone_done: 
                    if not ((dists[-20:] <= min_front_dist).any() or (dists[:20] <= min_front_dist).any()):
                        ang = 0
                        cone_cnt += 1
                        if cone_cnt > 10 and cone_start: cone_done = True
                    elif (dists[min_left:max_left] <= min_lr_dist).any():
                        spd = basic_spd / 3
                        cone_cnt = 0
                        if not (dists[min_right:max_right] <= min_lr_dist).any():
                            if ang < 0: ang = 0
                            ang = min(ang + angle_increment, 100)
                        else: 
                            if min(dists[min_right:max_right]) < min(dists[min_left:max_left]):
                                if ang > 0: ang = 0
                                ang = max(ang - angle_increment, -100)
                            else:
                                if ang < 0: ang = 0
                                ang = min(ang + angle_increment, 100)
                    elif (dists[min_right:max_right] <= min_lr_dist).any():
                        spd = basic_spd / 3
                        cone_cnt = 0
                        if ang > 0: ang = 0
                        ang = max(ang - angle_increment, -100)
                    else:
                        cone_start = True
                        cone_cnt = 0
                elif sum(dists[-15:] <= 15) + sum(dists[:15] <= 15) >= 5:
                    if sum(dists[-15:] <= 5) + sum(dists[:15] <= 5) >= 10:
                        spd = 0
                        print('stop!')
                    elif sum(dists[265:330] <= 10) < 3 and lane_left or sum(dists[30:95] <= 10) < 3 and not lane_left:
                        if not lane_change:
                            print('lane change')
                            lane_before = lane_left
                        lane_change = 1
                    else:
                        if lane_change:
                            print('change fail')
                        lane_change = 0
        drive(angle=ang, speed=spd)
        time.sleep(0.1)
        
        cv2.waitKey(1)

#=============================================
# 메인함수를 호출합니다.
# start() 함수가 실질적인 메인함수입니다.
#=============================================
if __name__ == '__main__':
    start()
