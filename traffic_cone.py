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
from sensor_msgs.msg import Image
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import argparse

#=============================================
# 인수를 입력받아 실행 (angle_increment, drive_speed)
#=============================================

parser = argparse.ArgumentParser()

parser.add_argument('-i', '--angle_increment', help='angle 변화정도', default=12)
parser.add_argument('-s', '--drive_speed', help='운행 속도', default=8)

args = parser.parse_args()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
ranges = None  # 라이다 데이터를 담을 변수
motor = None  # 모터노드
motor_msg = XycarMotor()  # 모터 토픽 메시지
Fix_Speed = 10  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
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
def start(a_inc, spd):

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
    angle_increment = a_inc
    min_left, max_left = 10, 55
    min_right, max_right = 305, 350
    min_dist = 4

    #=========================================
    # ang -> 핸들 조향각(실제로 넘겨줄 값)
    # angle_increment -> 조향각 조정 시, 각 if 블록마다 증가하는 각도
    # min_left, max_left -> 좌측 장애물 탐지각(최소, 최대)
    # min_right, max_right -> 우측 장애물 탐지각(최소, 최대)
    # min_dist -> 좌, 우측 장애물 탐지 최소 거리
    #=========================================

    drive(angle=ang, speed=90.0)
    time.sleep(4)

    while not rospy.is_shutdown():
        #=========================================
        # dists array에 lidar 센서에서 받은 값을 저장
        # 각 if문에서 장애물을 탐지하여 조건에 맞는 동작을 수행
        #
        # 1. 먼저 정면기준 좌우 20도에 장애물이 없으면 직진
        #
        # 2. 만약 왼쪽에만 장애물이 있다면 우회전
        #
        # 3. 왼쪽과 오른쪽 모두 장애물이 있다면 
        #    더 가까운 장애물이 있는 쪽 반대편으로 회전
        # ex) 왼쪽 3m, 오른쪽 2m에 장애물이 있다면 좌회전
        #
        # 4. 오른쪽에만 장애물이 있다면 좌회전
        #=========================================
        
        if ranges is not None:
            dists = np.array([round(d,1) for d in ranges])
            if not ((dists[-20:] <= 5).any() or (dists[:20] <= 5).any()):
                ang = 0
            elif (dists[min_left:max_left] <= min_dist).any():
                if not (dists[min_right:max_right] <= min_dist).any():
                    if ang < 0: ang = 0
                    ang = min(ang + angle_increment, 100)
                else: 
                    if min(dists[min_right:max_right]) < min(dists[min_left:max_left]):
                        if ang > 0: ang = 0
                        ang = max(ang - angle_increment, -100)
                    else:
                        if ang < 0: ang = 0
                        ang = min(ang + angle_increment, 100)
            elif (dists[min_right:max_right] <= min_dist).any():
                if ang > 0: ang = 0
                ang = max(ang - angle_increment, -100)
            

            
        drive(angle=ang, speed=spd)
        time.sleep(0.1)
        
        cv2.waitKey(1)

#=============================================
# 메인함수를 호출합니다.
# start() 함수가 실질적인 메인함수입니다.
#=============================================
if __name__ == '__main__':
    start(int(args.angle_increment), float(args.drive_speed))
