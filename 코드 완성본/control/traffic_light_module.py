import cv2

#=================== 신호등 인식 ======================
# 카메라로부터 초록불 인식하면 차량 출발
# 마스크를 사용해 인식 능력 강화
#=====================================================

def traffic_light(image):
    height, width, _ = image.shape
    green_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_green = (45, 100, 100)
    upper_green = (85, 255, 255)
    green_mask = cv2.inRange(green_hsv, lower_green, upper_green)

    return cv2.countNonZero(green_mask) > 500
            
