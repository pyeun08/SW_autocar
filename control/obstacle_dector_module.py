import numpy as np
import cv2, rospy, time, os, math
from xycar_msgs.msg import XycarMotor
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
import matplotlib.pyplot as plt
import argparse

lane_change=False
lane_before=True

def obstacle_dector(dists,spd,ang,lane_left):
    global lane_change
    global lane_before

    if lane_change:
        if lane_before != lane_left:
            lane_change = False
        
        if lane_change:
            lane_before = lane_left
            spd *= 2

            if lane_left:
                ang = 30
            else: 
                ang = -30

    if sum(dists[-15:] <= 15) + sum(dists[:15] <= 15) >= 5:

        if sum(dists[265:330] <= 5) < 3 and lane_left or sum(dists[30:95] <= 5) < 3 and not lane_left:
            if not lane_change:
                print('lane change')
                lane_before = lane_left
            lane_change = 1
        else:
            if lane_change:
                print('stop change')
            lane_change = 0

    return spd, ang , lane_change