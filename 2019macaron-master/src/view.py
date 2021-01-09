#!/usr/bin/env python
#-*-coding:utf-8-*_

## talker demo that published std_msgs/ColorRGBA messages
## to the 'color' topic. To see these messages, type:
##   rostopic echo color
## this demo shows some of the more advanced APIs in rospy.

import sys, time

from cv_bridge import CvBridge, CvBridgeError
from scipy.ndimage import filters

import cv2
import numpy as np
import roslib

roslib.load_manifest('macaron')

import rospy

from matplotlib import pyplot as plt
from std_msgs.msg import String
from std_msgs.msg import MultiArrayDimension

from macaron.msg import Floats
from macaron.msg import Floats_for_mission

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

flaots = Floats()
flaots_for_mission = Floats_for_mission()


cam = cv2.VideoCapture(1)
while True:  # @@@@@@@@@@@@@@@@@@@@영상출력루프 시작@@@@@@@@@@@@@@@@@@@@@@
    ret, video = cam.read()
    cv2.imshow('video', video)