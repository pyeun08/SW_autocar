import numpy as np
import cv2, rospy, time, os, math
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
import matplotlib.pyplot as plt
import argparse

# def usbcam_callback(data):
#     global image
#     image = bridge.imgmsg_to_cv2(data, "bgr8")

# rospy.Subscriber("/usb_cam/image_raw/",Image,usbcam_callback, queue_size=1)

def traffic_light(image):
    height, width, _ = image.shape
    green_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_green = (45, 100, 100)
    upper_green = (85, 255, 255)
    green_mask = cv2.inRange(green_hsv, lower_green, upper_green)

    return cv2.countNonZero(green_mask) > 500
            
