#=================== 라바콘 회피 ======================
# 전방 장애물 탐지 거리 내에 라바콘 탐지 안 될 시 라바콘 회피 종료
# 좌측에서만 라바콘 탐지 시 핸들 조향각 증가(우회전)
# 우측에서만 라바콘 탐지 시 핸들 조향각 감소(좌회전)
# 좌우측 모두 라바콘 탐지 시 탐지 거리 비교 후 좌측 라바콘이 더 가까우면 증가, 우측 라바콘이 더 가까우면 감소하도록 핸들 조향각 변경
# 
# cone_start -> 라바콘 회피 모드 Flag
# cone_done -> 라바콘 회피를 끝냈으면 True
# cone_cnt -> 라바콘 회피 ON/OFF 전환 카운터
# min_left, max_left -> 좌측 장애물 탐지각(최소, 최대)
# min_right, max_right -> 우측 장애물 탐지각(최소, 최대)
# min_lr_dist -> 좌, 우측 장애물 탐지 최소 거리
# min_front_dist -> 전방 장애물 탐지 최소 거리
# basic_spd -> 기본 주행 속도
# angle_increment -> 조향각 조정 시, 각 if 블록마다 증가하는 각도
#==================================================

cone_start = False 
cone_done = False 
cone_cnt = 0
min_left, max_left = 10, 55
min_right, max_right = 305, 350
min_lr_dist = 4
min_front_dist = 8
basic_spd = 25
angle_increment = 12

def cone_detector(dists,spd,ang):
    
    global cone_start
    global cone_done 
    global cone_cnt 
    global min_left, max_left
    global min_right, max_right
    global min_lr_dist
    global min_front_dist
    global basic_spd
    global angle_increment
    
    if not ((dists[-20:] <= min_front_dist).any() or (dists[:20] <= min_front_dist).any()):
        
        ang = 0
        cone_cnt += 1
        if cone_cnt > 7 and cone_start: cone_done = True

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
    
    return spd, ang, cone_start, cone_done