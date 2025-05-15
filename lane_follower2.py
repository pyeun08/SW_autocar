#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import XycarMotor

class LaneFollower:
    def __init__(self):
        rospy.init_node('lane_follower', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.motor_pub = rospy.Publisher('/xycar_motor', XycarMotor, queue_size=1)

    def callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width, _ = image.shape

        # ROI 설정 (하단 40%만 사용)
        roi = image[int(height * 0.6):, :]

        # HSV 변환
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
        cmd = XycarMotor()
        cmd.speed = 10.0  # 전진 속도

        # 노란 점선 인식 여부 확인
        M_yellow = cv2.moments(yellow_mask)

        if M_yellow['m00'] > 0:
            cx_yellow = int(M_yellow['m10'] / M_yellow['m00'])

            # 노란 점선이 화면의 오른쪽 → 왼쪽 흰선 사용
            if cx_yellow > center_image:
                selected_white_mask = white_mask[:, :center_image]

                offset = 0
            else:
                # 노란 점선이 왼쪽 → 오른쪽 흰선 사용
                selected_white_mask = white_mask[:, center_image:]
                offset = center_image

            M_white = cv2.moments(selected_white_mask)

            if M_white['m00'] > 0:
                cx_white = int(M_white['m10'] / M_white['m00']) + offset

                # 노란 점선과 해당 흰선 사이의 중앙 계산
                center_x = (cx_yellow + cx_white) // 2
                error = center_x - center_image

                # 조향 명령 설정
                cmd.angle = (float(error) / 100) * 40

                #print(error)
                # 코너에서 속도 감소
                if abs(error)>=130:
                    cmd.speed = 2.0
                    # if error<0:
                    #     cmd.angle -= 20
                    # else:
                    #     cmd.angle += 20
                    print("코너로 인해 속도 2")
                elif abs(error)>=120:
                    cmd.speed = 3.0
                    # if error<0:
                    #     cmd.angle -= 15
                    # else:
                    #     cmd.angle += 15
                    print("코너로 인해 속도 3")
                elif abs(error)>=110:
                    cmd.speed = 4.0
                    # if error<0:
                    #     cmd.angle -= 12
                    # else:
                    #     cmd.angle += 12
                    print("코너로 인해 속도 4")
                elif abs(error)>=100:
                    cmd.speed = 5.0
                    # if error<0:
                    #     cmd.angle -= 10
                    # else:
                    #     cmd.angle += 10
                    print("코너로 인해 속도 5")
                elif abs(error)>=90:
                    cmd.speed=6.0
                    # if error<0:
                    #     cmd.angle -= 5
                    # else:
                    #     cmd.angle += 5
                    print("코너로 인해 속도 6")

            
        # 퍼블리시
        self.motor_pub.publish(cmd)


if __name__ == '__main__':
    try:
        follower = LaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
