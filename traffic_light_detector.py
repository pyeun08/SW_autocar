#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class TrafficLightDetector:
    def __init__(self):
        rospy.init_node('traffic_light_detector')
        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.cmd_pub = rospy.Publisher('/traffic_cmd', Twist, queue_size=1)
        self.bridge = CvBridge()
        self.start_signal_sent = False

    def image_callback(self, msg):
        if self.start_signal_sent:
            return  # 한 번만 판단

        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_blue = (45, 100, 100)
        upper_blue = (85, 255, 255)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        if cv2.countNonZero(mask) > 500:
            cmd = Twist()
            cmd.linear.x = 1  # 출발 허용
            self.cmd_pub.publish(cmd)
            self.start_signal_sent = True
            rospy.loginfo("🔵 파란불 감지 - 출발!")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    TrafficLightDetector().run()
