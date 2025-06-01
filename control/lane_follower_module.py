import numpy as np
import cv2, rospy, time, os, math
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
import matplotlib.pyplot as plt
import argparse
from collections import deque

lane_left = True
basic_spd = 25
avg_err = deque(maxlen=10)

def lane_follower(image, spd,ang, cone_start,cone_done,lane_change):
    global lane_left
    global basic_spd
    global avg_err
    height, width, _ = image.shape

    # ROI 설정 (하단 40%만 사용)
    roi = image[int(height * 0.6):, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

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

    return spd, ang, lane_left