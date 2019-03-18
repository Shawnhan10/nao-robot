# -*- coding:utf-8 -*-
from naoqi import ALProxy
import time
import almath
import math
import logging
import cv2
import random
import numpy as np

# IP及端口
IP = "172.16.55.81"
# IP = "127.0.0.1"
Port = 9559
# 加载所需模块
g_tts = ALProxy("ALTextToSpeech", IP, Port)  # 说话模块
g_motion = ALProxy("ALMotion", IP, Port)  # 移动模块
g_posture = ALProxy("ALRobotPosture", IP, Port)  # 姿势模块
g_memory = ALProxy("ALMemory", IP, Port)  # 内存管理模块
g_camera = ALProxy("ALVideoDevice", IP, Port)  # 摄像头管理模块
g_landmarkDetection = ALProxy("ALLandMarkDetection", IP, Port)  # landMark检测模块
g_tracker = ALProxy("ALTracker", IP, Port)  # 追踪模块
g_videoDevice=ALProxy("ALVideoDevice", IP, Port)
g_motion.angleInterpolationWithSpeed("LHand", 1, 0.2)
#停止函数，松杆并坐下
def stop(t = 2):
    global g_motion, g_tracker, g_landmarkDetection
    # 停止追踪
    g_tracker.stopTracker()
    g_tracker.unregisterAllTargets()
    landmarkInfo = g_landmarkDetection.getSubscribersInfo()
    for info in landmarkInfo:
        g_landmarkDetection.unsubscribe(info[0])
    # 松开杆
    g_motion.openHand("LHand")
    time.sleep(t)
    g_motion.closeHand("LHand")
    # g_motion.rest()
stop()
