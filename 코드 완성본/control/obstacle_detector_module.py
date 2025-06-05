#=================== 차량 회피 ======================
# 정면에 차량으로 인식되는 장애물이 나타나면,
# 왼쪽 차선 주행 & 오른쪽 차선에 장애물 없을 경우 or 오른쪽 차선 주행 & 왼쪽 차선에 장애물 없을 경우 차선 변경
#
# lane_change -> 차선 변경을 해야할 경우 True
# lane_before -> 변경 전 차선을 기억하는 변수
#==================================================

lane_change=False
lane_before=True

def obstacle_detector(dists,spd,ang,lane_left):
    global lane_change
    global lane_before

    if lane_change:
        if lane_before != lane_left:
            print('change success')
            lane_change = False
        
        if lane_change:
            lane_before = lane_left
            spd *= 2.5

            if lane_left:
                ang = 25
            else: 
                ang = -25

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

    return spd, ang , lane_change