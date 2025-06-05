import numpy as np
import cv2
from collections import deque

#=================== 차선 변경 ======================
# 왼쪽 차선 주행 상태라면 핸들 조향각을 오른쪽으로, 오른쪽 차선 주행 상태라면 왼쪽으로 변경
# 차선 변경 시 원활한 변경을 위해 속도 증가
# 현재의 차선과 과거의 차선을 비교해 달라지면 차선 변경 종료
#
# lane_left -> 차선이 왼쪽이면 True
# basic_spd -> 기본 주행 속도
# avg_err -> 오차의 평균을 구하기 위한 큐, 최근 오차 동향이 낮으면(직선코스) 속도를 빠르게
#
# 맨 아래 부분은 visualize code로 cv2 패키지를 사용하여 원래 roi(하단 40%) image와 masked image를 동시에 보여준다.
#==================================================

lane_left = True    
basic_spd = 25    
avg_err = deque(maxlen=10)    

def lane_follower(image, spd,ang, cone_start,cone_done,lane_change):
    global lane_left
    global basic_spd
    global avg_err
    height, width, _ = image.shape
    selected_white_mask = 0

    roi = image[int(height * 0.6):, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    yellow_lower = np.array([20, 100, 100])
    yellow_upper = np.array([30, 255, 255])
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

    white_lower = np.array([0, 0, 230])
    white_upper = np.array([180, 15, 255])
    white_mask = cv2.inRange(hsv, white_lower, white_upper)

    center_image = width // 2
    spd = basic_spd

    M_yellow = cv2.moments(yellow_mask)

    if M_yellow['m00'] > 0 and (not cone_start or cone_done):
        cx_yellow = int(M_yellow['m10'] / M_yellow['m00'])

        if cx_yellow > center_image:
            selected_white_mask = white_mask[:, :center_image]
            offset = 0

            lane_left = True
        else:
            selected_white_mask = white_mask[:, center_image:]
            offset = center_image

            lane_left = False

        M_white = cv2.moments(selected_white_mask)

        if M_white['m00'] > 0 and not lane_change:
            cx_white = int(M_white['m10'] / M_white['m00']) + offset


            center_x = (cx_yellow + cx_white) // 2
            error = center_x - center_image
            
            if not lane_change:
                if abs(error) > 50:
                    if len(avg_err) > 3:
                        avg_err.clear()
                else: 
                    avg_err.append(abs(error))
                
                if len(avg_err) > 8:
                    if np.mean(avg_err) < 20:
                        spd *= 2.5
                    elif np.mean(avg_err) < 35:
                        spd *= 2
                    elif np.mean(avg_err) < 50:
                        spd *= 1.5

            ang = (float(error) / 100) * 55

    yellow_mask_color = np.zeros((yellow_mask.shape[0], yellow_mask.shape[1], 3), dtype=np.uint8)
    yellow_mask_color[yellow_mask == 255] = (0, 255, 255)

    if isinstance(selected_white_mask, np.ndarray):
        zero_padding = np.zeros((selected_white_mask.shape[0], selected_white_mask.shape[1]), dtype=np.uint8)
        if lane_left:
            white_mask_select = np.hstack((selected_white_mask, zero_padding)) 
        else:
            white_mask_select = np.hstack((zero_padding, selected_white_mask)) 
        vis_white = cv2.resize(cv2.cvtColor(white_mask_select, cv2.COLOR_GRAY2BGR), (width//2, height//2))
    else:
        vis_white = cv2.resize(cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR), (width//2, height//2))

    vis_roi = cv2.resize(roi, (width//2, height//2))
    vis_yellow = cv2.resize(yellow_mask_color, (width//2, height//2))

    vis = vis_yellow + vis_white

    top_row = np.hstack((vis_roi, vis))
    cv2.imshow('ROI | Yellow&White Mask', top_row)

    return spd, ang, lane_left