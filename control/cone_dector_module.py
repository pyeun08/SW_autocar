import numpy as np
import cv2, rospy, time, os, math
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
import matplotlib.pyplot as plt
import argparse


cone_start = False
cone_done = False
cone_cnt = 0
min_left, max_left = 10, 55
min_right, max_right = 305, 350
min_lr_dist = 4
min_front_dist = 8
basic_spd = 25
angle_increment = 12

def cone_dector(dists,spd,ang):
    
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