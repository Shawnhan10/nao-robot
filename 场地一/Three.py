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
# IP = "192.168.43.207"
# IP = "172.16.55.81"
IP = "192.168.43.207"
Port = 9559

g_tts = g_motion = g_posture = g_memory = g_camera = g_landmarkDetection = g_tracker = g_videoDevice = None

# 步伐参数配置
g_moveConfig1 = [["MaxStepX", 0.04], ["MaxStepY", 0.13], ["MaxStepTheta", 0.4], ["MaxStepFrequency", 0.5],
                 ["StepHeight", 0.0155], ["TorsoWx", 0.0], ["TorsoWy", 0.0]]

g_moveConfig2 = [["MaxStepX", 0.04], ["MaxStepY", 0.13], ["MaxStepTheta", 0.4], ["MaxStepFrequency", 0.65],
                 ["StepHeight", 0.023], ["TorsoWx", 0.0], ["TorsoWy", 0.0]]

g_moveConfig = [["MaxStepX", 0.04], ["MaxStepY", 0.13], ["MaxStepTheta", 0.4], ["MaxStepFrequency", 0.55],
                ["StepHeight", 0.022], ["TorsoWx", 0.0], ["TorsoWy", 0.0]]
# LandMark及ball参数
landmark = {"size": 0.09}
ball = {'name': 'RedBall', 'diameter': 0.04}
headang = 15


# -----------------------------初始化相关-------------------------
# 加载所需模块
def loadModule(IP="127.0.0.1", Port=9559):
    global g_tts, g_motion, g_posture, g_memory, g_camera, g_landmarkDetection, g_tracker
    g_tts = ALProxy("ALTextToSpeech", IP, Port)  # 说话模块
    g_motion = ALProxy("ALMotion", IP, Port)  # 移动模块
    g_posture = ALProxy("ALRobotPosture", IP, Port)  # 姿势模块
    g_memory = ALProxy("ALMemory", IP, Port)  # 内存管理模块
    g_camera = ALProxy("ALVideoDevice", IP, Port)  # 摄像头管理模块
    g_landmarkDetection = ALProxy("ALLandMarkDetection", IP, Port)  # landMark检测模块
    g_tracker = ALProxy("ALTracker", IP, Port)  # 追踪模块
    g_videoDevice = ALProxy("ALVideoDevice", IP, Port)


# 日志配置
def logConfig(logName):
    # 初始化所需模块
    logging.basicConfig(level=logging.DEBUG,
                        format='%(asctime)s  %(message)s',
                        # format='%(asctime)s %(filename)s[line:%(lineno)d] %(levelname)s %(message)s',
                        datefmt='%Y-%m-%d %H:%M:%S',
                        filename='./log/' + logName + time.strftime("%Y-%m-%d %H-%M-%S") + '.log',
                        filemode='w')
    # 定义一个StreamHandler，将INFO级别或更高的日志信息打印到标准错误，并将其添加到当前的日志处理对象#
    console = logging.StreamHandler()
    console.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(name)-12s: %(levelname)-8s %(message)s')
    console.setFormatter(formatter)
    logging.getLogger('').addHandler(console)


# 初始化
def naoInit(logName="no log name", IP="127.0.0.1"):
    """开始前机器人的初始化"""
    try:
        loadModule(IP)
        logConfig(logName)
    except Exception, e:
        try:
            loadModule()
            logConfig(logName)
        except Exception, e:
            logging.info(e)
            exit(2)
    # 1、站起来
    global g_motion, g_posture
    # g_motion.wakeUp()
    # // StandInit动作
    g_posture.goToPosture("StandInit", 0.5)
    # 第一次准备拿杆动作
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.3, 0.2)
    g_motion.angleInterpolationWithSpeed("LShoulderPitch", 0.77, 0.2)
    g_motion.angleInterpolationWithSpeed("LShoulderRoll", -1.5, 0.2)
    g_motion.angleInterpolationWithSpeed("LElbowRoll", -0.9, 0.2)
    g_motion.angleInterpolationWithSpeed("LElbowYaw", -1.8, 0.2)

    # 拿杆
    g_motion.angleInterpolationWithSpeed("LHand", 1.0, 0.2)
    time.sleep(4)
    g_motion.angleInterpolationWithSpeed("LHand", 0.15, 0.2)
    g_motion.setStiffnesses("LHand", 1.0)


# ------第三场-----------
# def firstHitBallForThree():
#     """ 第三场第一次击球 """
#     global g_motion, g_memory
#     # 触摸右手击球
#     while True:
#         if g_memory.getData("HandRightRightTouched"):
#             g_motion.angleInterpolationWithSpeed("LWristYaw", 0.6, 0.1)
#             g_motion.angleInterpolationWithSpeed("LWristYaw", -0.38, 0.115)#-0.40.15,
#             g_motion.angleInterpolationWithSpeed("LWristYaw", 0.4, 0.05)
#             break
def firstHitBallForThree():
    """ 第三场第一次击球 """
    global g_motion, g_memory
    time.sleep(1)
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.5, 0.26)  # 0.6
    # 触摸右手击球
    while True:
        if g_memory.getData("HandRightRightTouched"):
            g_motion.angleInterpolationWithSpeed("LWristYaw", -0.3, 0.068)  # 本地0.2
            g_motion.angleInterpolationWithSpeed("LWristYaw", 0.4, 0.05)
            break


# ----------------------------走路和停止相关-----------------------
def actionBeforeMove():
    """   走路前的拿杆动作 ,打完球后收杆
    """
    global g_motion
    # // moveInit动作
    g_motion.moveInit()

    # // 打完球为走路做准备的动作
    g_motion.angleInterpolationWithSpeed("LShoulderRoll", 0.25, 0.2)
    g_motion.angleInterpolationWithSpeed("LElbowRoll", 0.0, 0.2)
    g_motion.angleInterpolationWithSpeed("LShoulderPitch", 1.5, 0.2)
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.3, 0.2)

    g_motion.angleInterpolationWithSpeed("RElbowRoll", 0.0, 0.2)
    g_motion.angleInterpolationWithSpeed("RShoulderPitch", 1.5, 0.2)
    # g_motion.angleInterpolationWithSpeed("RShoulderRoll", -0.1, 0.2)

    g_motion.setMoveArmsEnabled(False, False)


# 移动函数，包装了api的移动，同时添加了一些修正
def move(x=0.0, y=0.0, theta=0.0, config=g_moveConfig):
    """
      NAO移动：以FRAME_ROBOT坐标系为参照，theta为角度
      :param x: 前进后退 单位cm
      :param y: 左右移动 cm
      :param theta: 旋转角度，往左为正，单位度数
      :param config: 行走参数配置
      :return:
      """

    global g_motion
    g_motion.moveInit()
    try:
        # 如果传入为小数，强转
        x1 = int(x + 0.5)
        x2 = round(x)
        y1 = round(y)
        theta1 = round((theta + 3.2) * almath.TO_RAD, 2)
        # 行走前初始化
        g_motion.moveInit()
        # 如果是往前走，修正
        step1 = x1 / 56
        # step2 = x1 % 56 / 20
        step3 = x1 % 56
        adjustY = 5
        adjustTheta = 4
        logging.info("step1 %f" % step1)
        # logging.info("step2  %f" % step2)
        logging.info("step3  %f" % step3)
        # 走至指定位置，解决走路偏斜问题
        # 第一段，走50cm
        if x1 > 0:
            for i in range(step1):
                g_motion.moveTo(56 * 0.01, 0, 0,
                                config)  # 行走,第一个参数为沿X轴行走的距离（米）,第二个参数为沿y轴行走的距离，  #第三个为沿z轴旋转的弧度（-3.1415~3.1415），第四个传入“config”表示可配置行走的步态。
                g_motion.moveTo(0, 0, -adjustTheta * almath.TO_RAD, config)
                # g_motion.moveTo(0, -adjustY * 0.01, 0, config)
                if i == 2:
                    adjustY += 2
                    # adjustTheta += 1
            # 第二段 走20cm
            # for i in range(step2):
            #     g_motion.moveTo(20 * 0.01, 0, 0, config)
            #     # g_motion.moveTo(0, 0, -1 * almath.TO_RAD, config)
            g_motion.moveTo(step3 * 0.01, 0, 0, config)
            # g_motion.moveTo(0, 0, 1 * almath.TO_RAD, config)
        else:
            g_motion.moveTo(x1 * 0.01, 0, 0, config)
        g_motion.moveTo(0, y1 * 0.01, theta1, config)
        # 记录日志
        logging.info("---------------------move------------------")
        if x != 0.0:
            logging.info("X:::: " + str(x2) + "cm")
        elif y != 0.0:
            logging.info("Y::::" + str(y1) + "cm")
        else:
            logging.info("Z:::: " + str(theta) + "度")

    except Exception:
        logging.error("传入参数不合法！")


def move1(x=0.0, y=0.0, theta=0.0, config=g_moveConfig):
    '''
            NAO移动：以FRAME_ROBOT坐标系为参照，theta为角度
            :param x: 前进后退 单位cm
            :param y: 左右移动 cm
            :param theta: 旋转角度，往左为正，单位度数
            :param config: 行走参数配置
            :return:
            '''

    global g_motion
    g_motion.moveInit()
    try:
        # 如果传入为小数，强转
        x1 = int(x + 0.5)
        print(x1)
        x2 = round(x)
        y1 = round(y)
        theta1 = round((theta + 2) * almath.TO_RAD, 2)
        # 行走前初始化
        g_motion.moveInit()
        # 如果是往前走，修正
        step1 = x1 / 56
        # step2 = x1 % 56 / 20
        step3 = x1 % 56
        adjustY = 1
        adjustTheta = 9  # 4
        adjustTheta2 = -4  # 3
        print("step1 ", step1)
        # print("step2  ", step2)
        print("step3  ", step3)
        # 走至指定位置，解决走路偏斜问题
        # 第一段，走50cm
        if x1 > 0:
            for i in range(step1):
                g_motion.moveTo(56 * 0.01, 0, 0,
                                config)  # 行走,第一个参数为沿X轴行走的距离（米）,第二个参数为沿y轴行走的距离，  #第三个为沿z轴旋转的弧度（-3.1415~3.1415），第四个传入“config”表示可配置行走的步态。
                g_motion.moveTo(0, 0, -adjustTheta * almath.TO_RAD, config)  #
                # adjustTheta += 1
                if i == 1:
                    g_motion.moveTo(0, 4 * 0.01, 0, config)
                    g_motion.moveTo(0, 0, 3 * almath.TO_RAD, config)
                if i == 2:
                    g_motion.moveTo(0, 0, 5 * almath.TO_RAD, config)
            # 第二段 走20cm
            # for i in range(step2):
            #     g_motion.moveTo(20 * 0.01, 0, 0, config)1
            #     # g_motion.moveTo(0, 0, -1 * almath.TO_RAD, config)
            g_motion.moveTo(step3 * 0.01, 0, 0, config)
            # g_motion.moveTo(0, 0, 1 * almath.TO_RAD, config)
        else:
            g_motion.moveTo(x1 * 0.01, 0, 0, config)

        g_motion.moveTo(0, y1 * 0.01, theta1, config)
        # 记录日志
        logging.info("---------------------move------------------")
        if x != 0.0:
            logging.info("X:::: " + str(x2) + "cm")
        elif y != 0.0:
            logging.info("Y::::" + str(y1) + "cm")
        else:
            logging.info("Z:::: " + str(theta) + "度")

    except Exception:
        logging.error("传入参数不合法！")


def move2(x=0.0, y=0.0, theta=0.0, config=g_moveConfig1):
    """
    NAO移动：以FRAME_ROBOT坐标系为参照，theta为角度
    :param x: 前进后退 单位cm
    :param y: 左右移动 cm
    :param theta: 旋转角度，往左为正，单位度数
    :param config: 行走参数配置
    :return:
    """
    global g_motion
    # 如果传入为小数，强转
    g_motion.moveInit()
    try:
        x1 = int(x + 0.5)
        x2 = round(x)
        y1 = round(y)
        theta1 = round(theta * almath.TO_RAD, 2)
        # 行走前初始化
        g_motion.moveInit()
        # 如果是往前走，修正
        step1 = x1 / 56
        # step2 = x1 % 56 / 20
        step3 = x1 % 56
        adjustY = 3
        adjustTheta = 8
        logging.info(("step1 %f" % step1))
        # logging.info("step2  %f" % step2)
        logging.info("step3  %f" % step3)
        # 走至指定位置，解决走路偏斜问题
        # 第一段，走50cm
        if x1 > 0:
            for i in range(step1):
                g_motion.moveTo(56 * 0.01, 0, 0, config)
                g_motion.moveTo(0, 0, -adjustTheta * almath.TO_RAD, config)
                # g_motion.moveTo(0, -adjustY * 0.01, 0, config)
                # adjustY +=1
                # adjustTheta += 2
            # 第二段 走20cm
            # for i in range(step2):
            #     g_motion.moveTo(20 * 0.01, 0, 0, config)
            #     # g_motion.moveTo(0, 0, -1 * almath.TO_RAD, config)
            g_motion.moveTo(step3 * 0.01, 0, 0, config)
        else:
            g_motion.moveTo(x1 * 0.01, 0, 0, config)
        g_motion.moveTo(0, y1 * 0.01, theta1, config)
        # 记录日志
        logging.info("---------------------move------------------")
        if x != 0.0:
            logging.info("X:::: " + str(x2) + "cm")
        elif y != 0.0:
            logging.info("Y::::" + str(y1) + "cm")
        else:
            logging.info("Z:::: " + str(theta) + "度")
    except Exception, e:
        logging.error("传入参数不合法！")
        stop()



def moveForThree(x=0.0, y=0.0, theta=0.0, config=g_moveConfig2):
    """
    NAO移动：以FRAME_ROBOT坐标系为参照，theta为角度
    :param x: 前进后退 单位cm
    :param y: 左右移动 cm
    :param theta: 旋转角度，往左为正，单位度数
    :param config: 行走参数配置
    :return:
    """
    global g_motion
    # 如果传入为小数，强转
    g_motion.moveInit()
    try:
        x1 = int(x + 0.5)
        x2 = round(x)
        y1 = round(y)
        theta1 = round(theta * almath.TO_RAD, 2)
        # 行走前初始化
        g_motion.moveInit()
        # 如果是往前走，修正
        step1 = x1 / 56
        # step2 = x1 % 56 / 20
        step3 = x1 % 56
        adjustY = 3
        adjustTheta = 8
        logging.info(("step1 %f" % step1))
        # logging.info("step2  %f" % step2)
        logging.info("step3  %f" % step3)
        # 走至指定位置，解决走路偏斜问题
        # 第一段，走50cm
        if x1 > 0:
            for i in range(step1):
                g_motion.moveTo(56 * 0.01, 0, 0, config)
                g_motion.moveTo(0, 0, -adjustTheta * almath.TO_RAD, config)
                # g_motion.moveTo(0, -adjustY * 0.01, 0, config)
                # adjustY +=1
                # adjustTheta += 1
            # 第二段 走20cm
            # for i in range(step2):
            #     g_motion.moveTo(20 * 0.01, 0, 0, config)
            #     # g_motion.moveTo(0, 0, -1 * almath.TO_RAD, config)
            g_motion.moveTo(step3 * 0.01, 0, 0, config)
        else:
            g_motion.moveTo(x1 * 0.01, 0, 0, config)
        g_motion.moveTo(0, y1 * 0.01, theta1, config)
        # 记录日志
        logging.info("---------------------move------------------")
        if x != 0.0:
            logging.info("X:::: " + str(x2) + "cm")
        elif y != 0.0:
            logging.info("Y::::" + str(y1) + "cm")
        else:
            logging.info("Z:::: " + str(theta) + "度")
    except Exception, e:
        logging.error("传入参数不合法！")
        stop()


'''
def closingForHitBallForThree():
    """
    为第三场第二杆调整距离，根据实际情况进行的调整，目的是为了达到击球点后，机器人是正对前方的
    :return:
    """
    ack = False  # 调整判断
    xTimes = 1  # x连续调整次数
    yTimes = 1
    # 获取红球距离
    distance, headYawAngle, ballCoord = trackBall(headang)
    # 防止数据异常
    while ballCoord[2] < -40:
        distance, headYawAngle, ballCoord = trackBall(headang)

    while ballCoord[1] > 24:
        move(y=9)
        #yTimes += 1
        distance, headYawAngle, ballCoord = trackBall(headang)
        while ballCoord[2] < -40:
            distance, headYawAngle, ballCoord = trackBall(headang)
        if distance > 30:
            move(distance-20)
            ack = True
        #if yTimes > 2:
            # move(theta=yTimes * 3)
            #yTimes = 0

    if ack == True:
        distance, headYawAngle, ballCoord = trackBall(headang)
        while ballCoord[2] < -40:
            distance, headYawAngle, ballCoord = trackBall(headang)

        move(y=ballCoord[1] - 15)

    distance_1, headYawAngle, ballCoord = trackBall(headang)
    while ballCoord[2] < -50:
        distance_1, headYawAngle, ballCoord = trackBall(headang)

    # g_motion.angleInterpolationWithSpeed("HeadPitch",0, 0.1)
    while distance_1 > 30.0:
        move(x=15)
        #xTimes += 1
        distance_1, headYawAngle, ballCoord = trackBall(headang)

    if (distance_1 > 15):
        move(x=distance_1 - 11)
    #if xTimes >= 2:
        # move(theta=-xTimes*1.5)
        pass
    logging.info("细调")
    #logging.info("%dxTimes" % xTimes)

    for i in range(2):
        distance, headYawAngle, ballCoord = trackBallNoHead(headang)
        while ballCoord[2] < -50:
            distance, headYawAngle, ballCoord = trackBallNoHead(headang)
            logging.info("%fdistance " % distance)

        if (12 < ballCoord[1] < 14):
            if 11 < distance < 13:
                break
            # elif distance > 12:
            #     move(x=distance - 11)
            else:
                move(x=distance - 11)
        else:
            #move(x=distance - 11)
            move(y=ballCoord[1] - 11)
        # elif ballCoord[1] > 10:
        #     move(y=ballCoord[1] - 8)
        # elif 9.5 > ballCoord[1]:
        #     move(y=ballCoord[1] - 9)
    # 复原头
    g_motion.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.1)
    g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1)
    time.sleep(0.5)
    # if distance_1 < 35:
    #     move(theta=5)
'''
'''
def closingForHitBallForThree():
    """
    为第三场第二杆调整距离，根据实际情况进行的调整，目的是为了达到击球点后，机器人是正对前方的
    :return:
    """
    logging.info("---------------------closingForHitBallForThree---------------------")
    ack = False  # 调整判断
    # 获取红球距离
    distance, headYawAngle, ballCoord = trackBall(headang)
    # 防止数据异常
    while ballCoord[2] < -40 or distance < 0:
        distance, headYawAngle, ballCoord = trackBall(headang)

    while ballCoord[1] > 24:
        move(y=9)
        distance, headYawAngle, ballCoord = trackBall(headang)
        while ballCoord[2] < -40 or distance < 0:
            distance, headYawAngle, ballCoord = trackBall(headang)
        if distance > 30:
            move(distance-20)
            ack = True


    if ack == True:
        distance, headYawAngle, ballCoord = trackBall(headang)
        while ballCoord[2] < -40 or distance < 0:
            distance, headYawAngle, ballCoord = trackBall(headang)

        move(y=ballCoord[1] - 15)

    distance_1, headYawAngle, ballCoord = trackBall(headang)
    while ballCoord[2] < -50 or distance < 0:
        distance_1, headYawAngle, ballCoord = trackBall(headang)

    # g_motion.angleInterpolationWithSpeed("HeadPitch",0, 0.1)
    while distance_1 > 30.0:
        move(x=15)
        distance_1, headYawAngle, ballCoord = trackBall(headang)
        while ballCoord[2] < -50 or distance < 0:
            distance_1, headYawAngle, ballCoord = trackBall(headang)
    if (distance_1 > 15):
        move(x=distance_1 - 11)
        pass
    logging.info("细调")

    for i in range(4):
        distance, headYawAngle, ballCoord = trackBallNoHead(headang)
        while ballCoord[2] < -50 or distance < 0:
            distance, headYawAngle, ballCoord = trackBallNoHead(headang)

        if (12 < ballCoord[1] < 14):
            if 10 < distance < 12:#11~13
                break
            # elif distance > 12:
            #     move(x=distance - 11)
            else:
                move(x=distance - 11)#11
        else:
            move(y=ballCoord[1] - 13)  # 11
            if distance < 10 or distance>12:#11~13
                move(x=distance - 11)

        # elif ballCoord[1] > 10:
        #     move(y=ballCoord[1] - 8)
        # elif 9.5 > ballCoord[1]:
        #     move(y=ballCoord[1] - 9)
    # 复原头
    g_motion.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.1)
    g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1)
    time.sleep(0.5)
    # if distance_1 < 35:
    #     move(theta=5)
'''


def get_pic(n=0):
    global g_videoDevice
    g_videoDevice = ALProxy("ALVideoDevice", IP, Port)
    n = n
    g_motion.setStiffnesses("Head", 1.0)

    # subscribe top camera
    AL_kTopCamera = 0
    AL_kQVGA = 1  # 320*240
    AL_kBGRColorSpace = 13
    # create image
    width = 320
    height = 240
    name = str(random.random())
    g_motion.angleInterpolationWithSpeed("HeadPitch", 5 * almath.TO_RAD, 0.3)
    # g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.3)
    captureDevice = g_videoDevice.subscribeCamera(
        name, AL_kTopCamera, AL_kQVGA, AL_kBGRColorSpace, 10)
    # np.zeros返回一个给定形状和类型的用0填充的数组；
    get_image = np.zeros((height, width, 3), np.uint8)  # ,unit8（无符号的整数，unit8是0～255）
    # get image
    result = g_videoDevice.getImageRemote(captureDevice)

    if result is None:
        print 'cannot capture.'

    elif result[6] is None:
        print 'no image data string.'
        # print result[6];

    else:
        # translate value to mat
        # imgHeader our image binary to the openCV image
        # map第一个参数 function 以参数序列中的每一个元素调用 function 函数，返回包含每次 function 函数返回值的新列表。
        # ord函数返回字符串对应的 ASCII 数值
        values = map(ord, list(result[6]))
        i = 0
        for y in range(0, height):
            for x in range(0, width):
                get_image.itemset((y, x, 0), values[i + 0])
                get_image.itemset((y, x, 1), values[i + 1])
                get_image.itemset((y, x, 2), values[i + 2])
                i += 3
        # cv2.imshow('get_image', image)
        # cv2.imwrite('white_line%d.jpg' % n)
    try:
        g_videoDevice.releaseImage(captureDevice)
        g_videoDevice.unsubscribe(captureDevice)
    except Exception, e:
        logging.info("Error%s" % e)
    return get_image

def get_theta(img):
    x, y, n = img.shape
    # img = img[y / 2:y, 0:x]  # 截取图片下半部分
    # cv2.imshow('image', img)
    result = img.copy()
    get_line = []  # 记录数据
    # Convert the img to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply edge detection method on the image
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    # cv2.imshow('edges', edges)
    # This returns an array of r and theta1 values
    # lines = cv2.HoughLines(edges, 1, math.pi / 180, 74, max_theta=50)
    lines = cv2.HoughLines(edges, 1, math.pi / 180, 74)

    # print(len(lines))
    # The below for loop runs till r and theta1 values
    # are in the range of the 2d array
    if lines is None:
        return
    if len(lines) != 0:
        for line in lines:
            rho, theta1 = line[0]
            if (theta1 < (math.pi / 4 * 3.)) and (theta1 > (math.pi / 5 * 2)):  # 垂直直线
                # print(line)
                # 该直线与第一列的交点
                pt1 = (0, int(rho / math.sin(theta1)))
                # 该直线与最后一列的交点
                pt2 = (result.shape[1], int((rho - result.shape[1] * math.cos(theta1)) / math.sin(theta1)))
                # 绘制一条直线
                # cv2.line(result, pt1, pt2, (255), 1)
                get_line.append(line[0])
        # All the changes made in the input image are finally
        # written on a new image houghlines.jpg
        # cv2.imwrite('houghlines3.jpg', img)
        # cv2.imshow('result', result)
        print('line number', len(get_line))
    # if num>0:
    #     theta0=0
    #     for i in get_line:
    #         theta1 = i[1]* 180 / math.pi - 90
    #         if abs(theta1)>abs(theta0):
    #             theta0=theta1
    #     print('theta1=', theta0)  # 正数为往右转
    if len(get_line) > 0:
        final_theta = get_line[0][1] * 180 / math.pi - 90
        # print('theta1=', final_theta)  # 正数为往右转
    else:
        final_theta = None
        # print('lines not in sight')
    return final_theta

def line_in_right(my_total_angle):
    b = 0
    None_time = 0
    all_theta = []
    while b != 1:
        # 转头 3度
        head_theta0 = g_motion.getAngles("HeadYaw", True)[0]
        head_theta0 *= almath.TO_DEG
        print 'head_theta0', head_theta0
        head_theta0 += my_total_angle
        print 'my_total_angle', head_theta0
        g_motion.angleInterpolationWithSpeed("HeadYaw", head_theta0 * almath.TO_RAD, 0.2)
        image = get_pic()
        print 'get pic'
        theta = get_theta(image)
        if theta is None:
            if None_time == 3:
                print ('theta is None')
                move(theta=-6)
                deal_theta()
                return
            None_time += 1
            continue
        print ('theta',theta)
        # 获取头的角度 head_theta
        head_theta = g_motion.getAngles("HeadYaw", True)[0]
        all_theta.append((theta, head_theta))  # 得到的白线的角度，头的角度
        if theta > 0:
            print('b change')
            b = 1
    small_theta = 100
    for i in all_theta:
        if abs(i[0]) < small_theta:
            small_theta = i[1]
    # 根据头的角度正对
    print ('small_theta',small_theta)
    g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1)  # 复原头
    move(theta=small_theta)

def deal_theta():
    a = 30
    b = 0
    c = 0
    # image = cv2.imread('../pic/jin%d.jpg' % a); cv2.imshow('image', image)
    while b != 1:
        image = get_pic()
        theta1 = get_theta(image)
        print('theta1', theta1)
        if theta1 is None:
            print '距离太远，需要向右旋转'
            move(theta=-9)  # 转 6
            c = 0
            # time.sleep(3)
        else:
            if theta1 > 5:
                # if theta1 > 0:
                print('大于5需要向右旋转 %f' % theta1)  # theta为正
                move(theta=-8)
            elif theta1>3:
                print('大于3需要向右旋转 %f' % theta1)  # theta为正
                move(theta=-6)
            elif theta1 < 0:
                print 'goto line_in_right'
                line_in_right(3)
                print('find again')
                line_in_right(1)
            else:
                if theta1 > 1:
                    move(theta=-4)
                else:
                    print('theta1,正对over %f' % theta1)
                    b = 1


# def closingForHitBallForThree():
#     """
#     为第三场第二杆调整距离，根据实际情况进行的调整，目的是为了达到击球点后，机器人是正对前方的
#     :return:
#     """
#     logging.info("---------------------closingForHitBallForThree---------------------")
#     ack = False  # 调整判断
#     # 获取红球距离
#     distance, headYawAngle, ballCoord = trackBall(headang)
#     # 防止数据异常
#     time1=0
#     while ballCoord[2] < -40 or distance < 0:
#         distance, headYawAngle, ballCoord = trackBall(headang)
#         if time1>5:
#             break
#         time1+=1
#     move(theta=headYawAngle)
#     move(x=distance - 13)
#
#     logging.info("细调")
#
#     deal_theta()
#
#     for i in range(4):
#         logging.info("第 %d 次调整" % (i + 1))
#         temp = 0
#         distance, headYawAngle, ballCoord = trackBallNoHead(headang)
#         while ballCoord[2] < -50 or distance < 0:
#             distance, headYawAngle, ballCoord = trackBallNoHead(headang)
#             if temp == 4:
#                move1(y=5)
#             temp+=1
#
#         if (12 < ballCoord[1] < 13):#12~14
#             if 9 <= distance < 11:#10~13
#                 break
#             # elif distance > 12:
#             #     move(x=distance - 11)
#             else:
#                 move1(x=distance - 9)#11
#         else:
#             if i == 3:
#                 logging.info("End times")
#                 if ballCoord[1] > 15:
#                     move1(y=2)
#                 elif 14 < ballCoord[1] < 15:
#                     move1(y=1)
#                 elif 10 < ballCoord[1] < 12:
#                     move1(y=-2)
#                 elif ballCoord[1] < 10:
#                     move1(y=3)
#             else:
#                 move1(y=ballCoord[1] - 13)  # 11
#             if i >= 2:
#                 distance, headYawAngle, ballCoord = trackBallNoHead(headang)
#                 while ballCoord[2] < -40 or distance < 0:
#                     distance, headYawAngle, ballCoord = trackBallNoHead(headang)
#                 if distance < 9 or distance >= 11:  # 11~13
#                     move(x=distance - 9)#11
#     # 复原头
#
#     g_motion.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.1)
#     g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1)
#     time.sleep(0.5)
#     # if distance_1 < 35:
#     #     move(theta=5)


# def secondHitBallForThree():
#     """第三场第二次击球"""
#     global g_motion
#     # 出杆
#     # // 第二次击球的准备动作（超远击球）
#     g_motion.angleInterpolationWithSpeed("LWristYaw", 0.4, 0.05)
#     g_motion.angleInterpolationWithSpeed("LShoulderPitch", 0.5, 0.2)
#     g_motion.angleInterpolationWithSpeed("LElbowYaw", -1.8, 0.2)
#     g_motion.angleInterpolationWithSpeed("LElbowRoll", -0.83, 0.2)
#     g_motion.angleInterpolationWithSpeed("LWristYaw", 0.7, 0.05)
#     g_motion.angleInterpolationWithSpeed("LShoulderRoll", -1.5, 0.2)
#     g_motion.angleInterpolationWithSpeed("LShoulderPitch", 0.83, 0.05)#0.837
#
#     # time.sleep(2)
#     # // 击球（超远击球）
#     g_motion.angleInterpolationWithSpeed("LWristYaw", 1.2, 0.1)  # 1.2
#     g_motion.angleInterpolationWithSpeed("LWristYaw", -0.55, 0.18)  # 0.6 0.18
#     # g_motion.angleInterpolationWithSpeed("LWristYaw", -0.6, 0.17)
#     g_motion.angleInterpolationWithSpeed("LWristYaw", 0.4, 0.05)

def closingForHitBallForThree():
    """
    为第三场第二杆调整距离，根据实际情况进行的调整，目的是为了达到击球点后，机器人是正对前方的
    :return:
    """
    logging.info("---------------------closingForHitBallForThree---------------------")
    ack = False  # 调整判断
    # 获取红球距离
    distance, headYawAngle, ballCoord = trackBall(headang)
    # 防止数据异常
    time1 = 0
    while ballCoord[2] < -40 or distance < 0:
        distance, headYawAngle, ballCoord = trackBall(headang)
        if time1 > 5:
            break
        time1 += 1
    move(theta=headYawAngle)
    distance, headYawAngle, ballCoord = trackBall(headang)
    time1 = 0
    while ballCoord[2] < -40 or distance < 0:
        distance, headYawAngle, ballCoord = trackBall(headang)
        if time1 > 5:
            break
        time1 += 1
    move(x=distance - 13)

    logging.info("细调")

    deal_theta()

    for i in range(4):
        logging.info("第 %d 次调整" % (i + 1))
        temp = 0
        distance, headYawAngle, ballCoord = trackBallNoHead(headang)
        while ballCoord[2] < -50 or distance < 0:
            distance, headYawAngle, ballCoord = trackBallNoHead(headang)
            if temp == 3:
                move1(y=5)  # 球被杆子挡住
            temp += 1

        if (11.5 < ballCoord[1] < 12.5):  # 12~14
            if 9 <= distance < 11:  # 10~13
                break
            # elif distance > 12:
            #     move(x=distance - 11)
            else:
                move1(x=distance - 10)  # 11
        else:
            if i == 3:
                logging.info("End times")
                if ballCoord[1] >= 15:
                    move1(y=3)
                elif 14 <= ballCoord[1] < 15:
                    move1(y=2)
                elif 13 <= ballCoord[1] < 14:
                    move1(y=1)
                elif 10 <= ballCoord[1] < 11:
                    move1(y=-2)
                elif ballCoord[1] <= 9.5:
                    move1(y=-3)
            else:
                move1(y=ballCoord[1] - 12)  # 11
            if i >= 2:
                # distance, headYawAngle, ballCoord = trackBallNoHead(headang)
                # while ballCoord[2] < -40 or distance < 0:
                #     distance, headYawAngle, ballCoord = trackBallNoHead(headang)
                # if distance < 9 or distance >= 11:  # 11~13
                    move(x=distance - 10)  # 11
    # 复原头

    g_motion.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.1)
    g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1)
    time.sleep(0.5)
    # if distance_1 < 35:
    #     move(theta=5)


def secondHitBallForThree():
    """第三场第二次击球"""
    global g_motion
    # 出杆
    # // 第二次击球的准备动作（超远击球）
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.4, 0.05)
    g_motion.angleInterpolationWithSpeed("LShoulderPitch", 0.5, 0.2)
    g_motion.angleInterpolationWithSpeed("LElbowYaw", -1.8, 0.2)
    g_motion.angleInterpolationWithSpeed("LElbowRoll", -0.83, 0.2)
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.7,0.05)
    g_motion.angleInterpolationWithSpeed("LShoulderRoll", -1.5, 0.2)
    g_motion.angleInterpolationWithSpeed("LShoulderPitch", 0.83, 0.05)  # 0.837

    # time.sleep(2)
    # // 击球（超远击球）
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.6, 0.1)  # 1.2
    g_motion.angleInterpolationWithSpeed("LWristYaw", -0.55, 0.17)  # 0.6 0.18
    # g_motion.angleInterpolationWithSpeed("LWristYaw", -0.6, 0.17)
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.4, 0.05)


def secondHitBallForThree1():
    """第三场第二次击球"""
    global g_motion
    # 出杆
    # // 第二次击球的准备动作（超远击球）
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.4, 0.05)
    g_motion.angleInterpolationWithSpeed("LShoulderPitch", 0.5, 0.2)
    g_motion.angleInterpolationWithSpeed("LElbowYaw", -1.8, 0.2)
    g_motion.angleInterpolationWithSpeed("LElbowRoll", -0.83, 0.2)
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.7,0.05)
    g_motion.angleInterpolationWithSpeed("LShoulderRoll", -1.5, 0.2)
    g_motion.angleInterpolationWithSpeed("LShoulderPitch", 0.83, 0.05)  # 0.837

    # time.sleep(2)
    # // 击球（超远击球）
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.6, 0.1)  # 1.2
    g_motion.angleInterpolationWithSpeed("LWristYaw", -0.55, 0.25)  # 0.6 0.18
    # g_motion.angleInterpolationWithSpeed("LWristYaw", -0.6, 0.17)
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.4, 0.05)

# ----------------------------球相关-------------------------------------------------
# -------下摄像头找球--------------------
def trackBall(headPitch=0):
    # 全局变量
    logging.info("---------------------trackBall---------------------")
    global g_camera, g_tracker, g_motion, g_tts
    # 需返回的参数
    headYawAngle = -1  # 头转过的角度
    distance = -1  # 球的距离
    coord = [0.0, 0.0, 0.0]  # 球的坐标
    # 一些后面需要用的参数
    times = 0  # 扭头次数
    yawDegree = 60 * almath.TO_RAD
    # 避免识别到右手的红色
    if g_motion.getAngles("RWristYaw", False)[0] != 0.6:
        g_motion.angleInterpolationWithSpeed("RWristYaw", 0.6, 0.2)
    # 启用下摄像头
    print g_camera.isCameraOpen(0)
    g_camera.setActiveCamera(1)
    # 注册追踪物
    g_tracker.registerTarget(ball['name'], ball['diameter'])
    # 设置追踪模式
    g_tracker.setMode("Head")
    # 启动追踪
    g_tracker.track(ball['name'])
    # 睡眠2秒,给机器人追踪时间
    time.sleep(2)
    # 找球
    # 将机器人头设置到指定位置
    if headPitch != 0 and g_motion.getAngles("HeadPitch", False)[0] != headPitch:
        g_motion.angleInterpolationWithSpeed("HeadPitch", headPitch * almath.TO_RAD, 0.15)
    while True:
        time.sleep(1)
        # 追踪到球
        if len(g_tracker.getTargetPosition(2)) == 3:
            # g_tts.say("got it")
            # time.sleep(1)
            # 以脚部坐标系获取坐标
            coord = g_tracker.getTargetPosition(2)
            for i in range(2):  # 重复赋值两次?
                coord = g_tracker.getTargetPosition(2)
            # 将坐标转换为cm
            for i in range(len(coord)):
                coord[i] = coord[i] * 100
            # 获取头转的角度
            headYawAngle = g_motion.getAngles("HeadYaw", True)[0] * almath.TO_DEG
            headPitchAngle = g_motion.getAngles("HeadPitch", True)[0] * almath.TO_DEG
            # 打印
            logging.info("球的坐标x:" + str(coord[0]) + " y: " + str(coord[1]) + " z: " + str(coord[2]) +
                         "headYawAngle:" + str(headYawAngle) + " headPitchAngle:" + str(headPitchAngle))

            distance = getRealDisForBall(coord[0], coord[1], coord[2], headYawAngle, headPitchAngle)
            logging.info("nao与球的距离 ::" + str(distance) + "    角度::" + str(headYawAngle))
            # 跳出循环
            break
        # 没找到球
        else:
            # 判断扭头次数，扫描3遍
            if times < 9:
                # 先扭头往右边观察
                if times > 5:
                    # g_camera.setActiveCamera(0)
                    g_motion.setAngles("HeadPitch", 13 * almath.TO_RAD, 0.15)
                if times % 3 == 0:
                    g_motion.angleInterpolationWithSpeed("HeadYaw", 0, 0.2)
                    times += 1
                    time.sleep(0.5)
                elif times % 3 == 2:
                    g_motion.angleInterpolationWithSpeed("HeadYaw", -yawDegree,0.2)
                    times += 1
                    time.sleep(0.5)
                else:
                    g_motion.angleInterpolationWithSpeed("HeadYaw", yawDegree, 0.2)
                    times += 1
                    time.sleep(0.5)
                    # logging.info("转头次数:" + str(times))
                print("转头次数:" + str(times))
            else:
                g_tts.say("ball not in sight")
                break

    # 停止追踪
    g_tracker.stopTracker()
    g_tracker.unregisterTarget(ball['name'])

    # 复原头
    g_motion.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.15)
    g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.15)
    # 输出球坐标和角度
    logging.debug("球修正后坐标cm: " + str(coord))
    # 返回距离和角度
    print('%distance' % distance)
    return round(distance, 2), round(headYawAngle, 4), coord

# -------下摄像头第二次找球---------------
def trackBallNoHead(headPitch=0):
    # 全局变量
    logging.info("---------------------trackBallNoHead---------------------")
    global g_camera, g_tracker, g_motion, g_tts
    # 需返回的参数
    headYawAngle = -1  # 头转过的角度
    distance = -1  # 球坐标
    coord = [0.0, 0.0, 0.0]
    # 一些后面需要用的参数
    times = 0  # 扭头次数
    yawDegree = 60 * almath.TO_RAD
    # 避免识别到右手的红色
    if g_motion.getAngles("RWristYaw", False)[0] != 0.6:
        g_motion.angleInterpolationWithSpeed("RWristYaw", 0.6, 0.2)
    # 启用下摄像头
    g_camera.setActiveCamera(1)
    # 注册追踪物
    g_tracker.registerTarget(ball['name'], ball['diameter'])
    # 设置追踪模式
    g_tracker.setMode("Head")
    # 启动追踪
    g_tracker.track(ball['name'])
    # 睡眠2秒,给机器人追踪时间
    time.sleep(2)
    # 找球
    # 将机器人头设置到指定位置
    if headPitch != 0 and g_motion.getAngles("HeadPitch", False)[0] != headPitch:
        g_motion.angleInterpolationWithSpeed("HeadPitch", headPitch * almath.TO_RAD, 0.15)
    while True:
        time.sleep(1)
        # 追踪到球
        if len(g_tracker.getTargetPosition(2)) == 3:
            g_tts.say("get it")
            # 以脚部坐标系获取坐标
            coord = g_tracker.getTargetPosition(2)
            for i in range(2):
                coord = g_tracker.getTargetPosition(2)
            # 将坐标转换为cm
            for i in range(len(coord)):
                coord[i] = coord[i] * 100
            # 获取头转的角度
            headYawAngle = g_motion.getAngles("HeadYaw", True)[0] * almath.TO_DEG
            headPitchAngle = g_motion.getAngles("HeadPitch", True)[0] * almath.TO_DEG
            # 打印
            logging.info("球的坐标x:" + str(coord[0]) + " y: " + str(coord[1]) + " z: " + str(coord[2]) +
                         "headYawAngle:" + str(headYawAngle) + " headPitchAngle:" + str(headPitchAngle))

            distance = getRealDisForBall(coord[0], coord[1], coord[2], headYawAngle, headPitchAngle)
            logging.info("nao与球的距离::" + str(distance) + "    角度::" + str(headYawAngle))
            # 跳出循环
            break
        # 没找到球
        else:
            # 判断扭头次数，扫描3遍
            if times < 6:
                # 先扭头往右边观察
                if times > 2:
                    # g_camera.setActiveCamera(0)
                    g_motion.setAngles("HeadPitch", 13 * almath.TO_RAD, 0.15)
                if times % 3 == 0:
                    g_motion.angleInterpolationWithSpeed("HeadYaw", 0, 0.2)
                    times += 1
                    time.sleep(0.5)
                elif times % 3 == 2:
                    g_motion.angleInterpolationWithSpeed("HeadYaw", -yawDegree, 0.2)
                    times += 1
                    time.sleep(0.5)
                else:
                    g_motion.angleInterpolationWithSpeed("HeadYaw", yawDegree, 0.2)
                    times += 1
                    time.sleep(0.5)
                    # logging.info("转头次数:" + str(times))
                print("转头次数:" + str(times))
            else:
                g_tts.say("ball not in sight")
                break

    # 停止追踪
    g_tracker.stopTracker()
    g_tracker.unregisterTarget(ball['name'])

    # 复原头
    # g_motion.angleInterpolationWithSpeed("HeadPitch",headPitch * almath.TO_RAD, 0.1)
    g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1)

    # 输出球坐标和角度
    logging.debug("球修正后坐标cm: " + str(coord))
    # 返回距离和角度
    return distance, headYawAngle, coord


def trackBallforthree(headPitch=0):
    # 全局变量
    logging.info("---------------------trackBall---------------------")
    global g_camera, g_tracker, g_motion, g_tts
    # 需返回的参数
    headYawAngle = -1  # 头转过的角度
    distance = -1  # 球的距离
    coord = [0.0, 0.0, 0.0]  # 球的坐标
    # 一些后面需要用的参数
    times = 0  # 扭头次数
    yawDegree = 60 * almath.TO_RAD
    # 避免识别到右手的红色
    if g_motion.getAngles("RWristYaw", False)[0] != 0.6:
        g_motion.angleInterpolationWithSpeed("RWristYaw", 0.6, 0.2)
    # 启用下摄像头
    print g_camera.isCameraOpen(0)
    g_camera.setActiveCamera(1)
    # 注册追踪物
    g_tracker.registerTarget(ball['name'], ball['diameter'])
    # 设置追踪模式
    g_tracker.setMode("Head")
    # 启动追踪
    g_tracker.track(ball['name'])
    # 睡眠2秒,给机器人追踪时间
    time.sleep(2)
    # 找球
    # 将机器人头设置到指定位置
    if headPitch != 0 and g_motion.getAngles("HeadPitch", False)[0] != headPitch:
        g_motion.angleInterpolationWithSpeed("HeadPitch", headPitch * almath.TO_RAD, 0.1)
    while True:
        time.sleep(1)
        # 追踪到球
        if len(g_tracker.getTargetPosition(2)) == 3:
            # g_tts.say("got it")
            # time.sleep(1)
            # 以脚部坐标系获取坐标
            coord = g_tracker.getTargetPosition(2)
            for i in range(2):  # 重复赋值两次?
                coord = g_tracker.getTargetPosition(2)
            # 将坐标转换为cm
            for i in range(len(coord)):
                coord[i] = coord[i] * 100
            # 获取头转的角度
            headYawAngle = g_motion.getAngles("HeadYaw", True)[0] * almath.TO_DEG
            headPitchAngle = g_motion.getAngles("HeadPitch", True)[0] * almath.TO_DEG
            # 打印
            logging.info("球的坐标x:" + str(coord[0]) + " y: " + str(coord[1]) + " z: " + str(coord[2]) +
                         "headYawAngle:" + str(headYawAngle) + " headPitchAngle:" + str(headPitchAngle))

            distance = getRealDisForBall(coord[0], coord[1], coord[2], headYawAngle, headPitchAngle)
            logging.info("nao与球的距离 ::" + str(distance) + "    角度::" + str(headYawAngle))
            # 跳出循环
            break
        # 没找到球
        else:
            if times > 6:
                break
            # 判断扭头次数，扫描3遍
            if times % 3 == 0:
                g_motion.angleInterpolationWithSpeed("HeadYaw", 0, 0.4)
                times += 1
                time.sleep(0.5)
            elif times % 3 == 2:
                g_motion.angleInterpolationWithSpeed("HeadYaw", -yawDegree, 0.4)
                times += 1
                time.sleep(0.5)
            else:
                g_motion.angleInterpolationWithSpeed("HeadYaw", yawDegree, 0.4)
                times += 1
                time.sleep(0.5)
                # logging.info("转头次数:" + str(times))
            print("转头次数:" + str(times))

        # g_tts.say("ball not in sight")
    # 停止追踪
    g_tracker.stopTracker()
    g_tracker.unregisterTarget(ball['name'])

    # 复原头
    g_motion.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.15)
    g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.15)
    # 输出球坐标和角度
    logging.debug("球修正后坐标cm: " + str(coord))
    # 返回距离和角度
    print('%distance' % distance)
    return round(distance, 2), round(headYawAngle, 4), coord


def getRealDisForBall(x, y, z, yaw, pitch):
    """
    根据api返回的数据，计算真正的距离
    :param x:
    :param y:
    :param z:
    :param yaw:
    :param pitch:
    :return:
    """
    # 初始化一些数据
    print 'z', z
    z = abs(z)
    # 获取
    dis1 = math.sqrt(x ** 2 + y ** 2)
    dis_z = dis1 - z
    logging.info("dis1:%f" % dis1)
    logging.info("dis_z:%f" % dis_z)
    # 以下处理通过分析数据获得，基本依照2017-7-19的数据（注:现在需要改进）
    if dis1 < 10:
        # 未找到球
        return
    elif dis1 < 20.59 and z < 25 or (dis_z < 5 and z < 25):
        # 距离在0-5cm之间
        # if z > 20:
        #     distance = dis1 - 15
        # else:
        #     distance = dis1 - 14
        # 另一种处理方式
        distance = (dis1 - (z - 13.31) / 3.92 - 14.47) / 0.918
    elif dis1 < 27.12 or dis_z < 9.61:
        # 5 -10
        # logging.info("5-10")
        logging.debug("5-10")
        distance = (dis1 - (z - 13.19) / 2.55 - 19.19) / 1.262 + 5
    elif dis1 < 40.47 or dis_z < 9.94:
        # 10-15
        logging.debug("10-15")
        distance = (dis1 - (z - 20.23) / 2.12 - 28.36) / 1.424 + 10
    elif dis1 < 51.43 or (dis_z < 17.69 and z < 50):
        # 15-20
        logging.debug("15-20")
        distance = (dis1 - (z - 25.17) / 1.67 - 40) / 1.582 + 15
    elif dis1 < 53.14 or dis_z < 30:
        # 20- 25
        logging.debug("20-25")
        distance = (dis1 - (z - 25) / 1.37 - 46.52) / 1.6 + 20
    elif dis1 < 58.94 or dis_z < 38.07:
        # 25- 30
        logging.debug("25-30")
        distance = (dis1 - (z - 15.80) / 1.13 - 47.55) / 1.416 + 25
    elif dis1 < 62.83 or dis_z < 46:
        # 30- 35
        logging.debug("30-35")
        distance = (dis1 - (z - 16.20) / 1.08 - 55) / 1.404 + 30
    elif dis1 < 85.20 or dis_z < 54.15:
        # 35 - 40
        logging.debug("35-40")
        distance = (dis1 - (z - 22.22) / 0.91 - 67.5) / 1.6 + 35
    elif dis_z < 67:
        # 40-45
        logging.debug("40-45")
        distance = (dis1 - (z - 18.46) / 0.86 - 71.74) / 1.358 + 40
    elif (dis1 < 120 and dis_z < 75) or (dis1 / z < 2.7 and dis_z < 85):
        # 45 - 50
        logging.debug("45-50")
        distance = (dis1 - (z - 27.21) / 0.78 - 88.73) / 1.542 + 45
    elif dis_z < 81.5 or (dis1 / z < 3.35 and dis1 < 116.5):
        # 50 - 55
        logging.debug("50-55")
        distance = (dis1 - (z - 22.91) / 0.72 - 92.17) / 1.488 + 50
    elif (dis_z < 87.5) or (dis1 > 152 and dis_z < 100.5):
        # 55-60
        logging.debug("55-60")
        distance = (dis1 - (z - 31.12) / 0.72 - 112.47) / 1.212 + 55
    elif dis_z < 96.5 or (dis1 > 150 and dis_z < 111.5 and z > 45):
        # 60-65
        logging.debug("60-65")
        distance = (dis1 - (z - 31.60) / 0.64 - 119) / 1.8 + 60
    elif dis_z < 106 or (dis1 > 158 and dis_z < 119.5 and z > 44):
        # 65 - 70
        logging.debug("65-70")
        distance = (dis1 - (z - 32.86) / 0.60 - 130.26) / 1.714 + 65
    elif dis_z < 123.5 or (dis1 > 163 and dis1 / z < 3.8):
        # 70-75
        logging.debug("70-75")
        distance = (dis1 - (z - 27.68) / 0.54 - 130.755) / 2.099 + 70
    else:
        # 75 -
        logging.debug("75-")
        distance = (dis1 - (z - 22.34) / 0.61 - 134.52) / 2.408 + 75
    logging.info("distance:%f" % distance)
    return distance


# ------------上摄像头找球，使用opencv部分，此处不做注释-------------------
def searchBallUseTop():
    """
    适用上摄像头进行找球
    :return:
    """
    initAngle = 40  # 从左边开始扫描
    endAngle = -40
    currAngle = initAngle
    distance = -1
    distance, angle = findRedBallUseTop()
    # cv2.waitKey(0)
    logging.info("----------------searchBallUseTop-------------------------")
    if distance > 0:
        g_tts.say("find RedBall")
        headYawAngle = g_motion.getAngles("HeadYaw", True)[0]  # getAngles返回的是含一个数值的列表，所以取列表的第一个元素
        logging.info("headYawAngle:%f" % (headYawAngle * almath.TO_DEG))
        angle = (angle + headYawAngle) * almath.TO_DEG
        logging.info("distance: %f" % (distance) + "______SUMangle: %f" % (angle))
        return distance, angle
    while currAngle >= endAngle:
        # 找到球
        g_motion.angleInterpolationWithSpeed("HeadYaw", currAngle * almath.TO_RAD, 0.4)
        distance, angle = findRedBallUseTop()
        if distance > 0:
            g_tts.say("got it")
            break
        # time.sleep(0.5)
        currAngle -= 40
    headYawAngle = g_motion.getAngles("HeadYaw", True)[0]
    logging.info("headYawAngle= %f" % (headYawAngle * almath.TO_DEG))
    # logging.info("headYawAngle:%f" %headYawAngle* almath.TO_DEG)
    angle = (angle + headYawAngle) * almath.TO_DEG  # 主光轴与小球的偏角加上机器人头部左右方向的偏角，结果为小球与机器人正面的夹角

    logging.info("distance: %f" % (distance) + "_______SUMangle: %f" % (angle))
    # move(theta=angle * almath.TO_DEG)
    return distance, angle


def findRedBallUseTop():
    '''
    使用上摄像头找球
    :return:
    '''
    distance = 0
    angle = 0
    global g_videoDevice, yuan, zhao
    g_videoDevice = ALProxy("ALVideoDevice", IP, Port)
    g_motion.setStiffnesses("Head", 1.0)

    # subscribe top camera
    AL_kTopCamera = 0
    AL_kQVGA = 1  # 320*240
    AL_kBGRColorSpace = 13
    # create image
    width = 320
    height = 240
    name = str(random.random())
    g_motion.angleInterpolationWithSpeed("HeadPitch", 5 * almath.TO_RAD, 0.3)
    # g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.3)
    captureDevice = g_videoDevice.subscribeCamera(
        name, AL_kTopCamera, AL_kQVGA, AL_kBGRColorSpace, 10)

    for times in xrange(10):
        print times
        # np.zeros返回一个给定形状和类型的用0填充的数组；
        image = np.zeros((height, width, 3), np.uint8)  # ,unit8（无符号的整数，unit8是0～255）
        # get image
        result = g_videoDevice.getImageRemote(captureDevice)

        if result == None:
            print 'cannot capture.'

        elif result[6] == None:
            print 'no image data string.'
            # print result[6]

        else:
            # translate value to mat
            # imgHeader our image binary to the openCV image
            # map第一个参数 function 以参数序列中的每一个元素调用 function 函数，返回包含每次 function 函数返回值的新列表。
            # ord函数返回字符串对应的 ASCII 数值
            values = map(ord, list(result[6]))
            i = 0
            for y in range(0, height):
                for x in range(0, width):
                    image.itemset((y, x, 0), values[i + 0])
                    image.itemset((y, x, 1), values[i + 1])
                    image.itemset((y, x, 2), values[i + 2])
                    i += 3
            # cv2.imshow('image',image)


            image = image[height / 5*3:height, 0:width]
            cv2.imwrite('./pic/yuan'+time.strftime("%Y-%m-%d %H-%M-%S")+'.jpg', image)

            # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # . cv2.cvtColor(变换颜色空间以便处理图像): 第一个参数要进行颜色空间变换的原图像，第二个为变换后存储图像格式
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            # 需要对一个像素的周围的像素给予更多的重视。因此，
            # 可通过分配权重来重新计算这些周围点的值。这可通过高斯函数（钟形函数，即喇叭形数）的权重方案来解决。

            # blur1 = cv2.GaussianBlur(hsv, (0, 0), 1)  # 高斯模糊
            #cv2.imwrite('./pic/blur1' + time.strftime("%Y-%m-%d %H-%M-%S") + '.jpg', blur1)

            # define range of red color in HSV
            # lower_red = np.array([140, 65, 65])  # 新亮#140#65,65
            # upper_red = np.array([180, 255, 255])  # 新亮

            # lower_red = np.array([156, 43, 46])
            # upper_red = np.array([180, 255, 255])

            # lower_red = np.array([156, 43, 46])
            lower_red = np.array([156, 130, 70])#原飽和度43
            upper_red = np.array([180, 255, 255])

            # lower_red = np.array([156, 65, 65])#旧亮
            # upper_red = np.array([180, 255, 255])
            # lower_red = np.array([0,42,46])  # 暗
            # upper_red = np.array([10, 255, 255])
            # Threshlod the HSV image to get only red colors
            # 就是将低于lower_red和高于upper_red的部分分别变成0（黑色），lower_red～upper_red之间的值变成255（白色）
            mask = cv2.inRange(hsv, lower_red, upper_red)
            # 使用位“与”运算来提取图像精确的边界，把白色区域从图中抠出来
            # Bitwise-AND mask and original image
            red = cv2.bitwise_and(hsv, hsv, mask=mask)
            # cv2.imshow('mask',mask)
            # 形态学处理
            img1 = cv2.cvtColor(red, cv2.COLOR_HSV2BGR)
            img2 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            # threshold：固定阈值二值化，参数2为阈值： 参数img_morph:maxval 当像素值超过了阈值（或者小于阈值，
            # 根据参数4:type（赋值方法）来决定）所赋予的值，threshold有两个返回值，第一个为得到的阈值值，第二个就是阈值化后的图像
            _, thresh = cv2.threshold(img2, 127, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            cv2.imwrite('./pic/thresh' + time.strftime("%Y-%m-%d %H-%M-%S") + '.jpg', thresh)
            #cv2.imshow('thresh',thresh)
            # 做一些形态学操作，去一些小物体的干扰
            # 开运算函数：morphologyEx,先进行腐蚀再进行膨胀就叫做开运算，被用来去除噪声
            img_morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, (1, 1))
            cv2.imwrite('./pic/img_morph' + time.strftime("%Y-%m-%d %H-%M-%S") + '.jpg', img_morph)
            # cv2.imshow('img_morph', img_morph)
            # cv2.erode(img_morph, (1, 1), img_morph, iterations=1)
            cv2.dilate(img_morph, (1, 1), img_morph, iterations=1)
            #cv2.imwrite('./pic/img_morph2' + time.strftime("%Y-%m-%d %H-%M-%S") + '.jpg', img_morph)
            blur = cv2.GaussianBlur(img_morph, (0, 0), 1)
            # cv2.imshow('blur', blur)
            cv2.imwrite('./pic/blur' + time.strftime("%Y-%m-%d %H-%M-%S") + '.jpg', blur)
            # threshold image
            ret, threshed_img = cv2.threshold(blur, 127, 255, cv2.THRESH_BINARY)#127#30
            # if times==0 or times==1:
            cv2.imwrite('./pic/chuli' + time.strftime("%Y-%m-%d %H-%M-%S") + '.jpg', threshed_img)
            # ret, threshed_img = blur
            # find contours and get the external one
            contours, hier = cv2.findContours(threshed_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # cv2.imshow('threshed_img', threshed_img)
            # lunkuo=cv2.drawContours(threshed_img, contours, -1, (255, 0, 0), 3)
            # cv2.imshow('lunkuo',lunkuo)
            # if not find contours, then print"Not find ball"
            if len(contours) == 0 and times > 2:
                print 'Not find ball'
                FindBall = 0
                # return FindBall
                break
            else:
                n = 0
                minContourArea = 3.0  # 轮廓面积
                maxContourArea = 90  # 100.0
                contourTempArea = 0.0

                for c in contours:
                    # get the bounding rect
                    x, y, w, h = cv2.boundingRect(c)
                    y+=height/5*3
                    print 'x:', x
                    print 'y:', y
                    print 'w:', w
                    print 'h', h
                    # 图片下方的发现点,75cm-190cm
                    contoursArea = cv2.contourArea(c)
                    print "Area:", contoursArea
                    if minContourArea <= contoursArea <= maxContourArea:
                        # print "Area_right:", contoursArea
                        useSensors = False
                        headAngle = g_motion.getAngles('HeadPitch', useSensors)
                        cv2.rectangle(image, (x, y-height/5*3), (x + w, y-height/5*3+ h), (0, 255, 0), 1)
                        # cv2.imshow('image2',image)
                        # if times == 0 or times == 1:
                        cv2.imwrite('./pic/zhao'+time.strftime("%Y-%m-%d %H-%M-%S")+'.jpg', image)
                        center_x = x + w / 2
                        center_y = y + h / 2
                        logging.info("center_x: %f" % (center_x) + "______________centery: %f" % (center_y))
                        distance, angle = calDistanceAndAngle(center_x, center_y, width, height, headAngle[0],
                                                              AL_kTopCamera)
                        # cv2.imshow('blur1', blur1)
                        # 在下方发现有点之后就停止，因为在腐蚀的时候可能会将一个圆分为几块会出现这种情况
                        break
            if distance > 0:
                break
                # k = cv2.waitKey(0) & 0xFF
                #
                # if k == 27:
                #     break

                # cv2.destroyAllWindows()

    try:
        g_videoDevice.releaseImage(captureDevice)
        g_videoDevice.unsubscribe(captureDevice)
    except Exception, e:
        print "Error", e
        pass
    finally:
        return distance, angle



def calDistanceAndAngle(cxnum, rynum, colsum, rowsum, Head_angle, cameraID):
    """
    上摄像头找到球后计算距离和角度
    :param cxnum:
    :param rynum:
    :param colsum:#宽度
    :param rowsum:#高度
    :param Head_angle:#头向上或向下偏的角度,向上为负数
    :param cameraID:
    :return:
    """
    distx = colsum / 2 - cxnum  # 小球与主光轴在水平方向的偏移量
    disty = rynum - rowsum / 2  # 小球与主光轴在竖直方向的偏移量
    logging.info("distx: %f" % (distx) + "______________disty: %f" % (disty))

    Picture_angle = disty * 47.64 / 240  # 小球与主光轴的竖直偏角
    logging.info("Picture_angle: %f" % Picture_angle)

    if cameraID == 0:  # 上摄像头
        # 摄像头距地面的距离减去球的半径
        h = 0.36#0.34  # 0.475#0.3405
        Camera_angle =4.7145 #4.7145  # 12.624#12.3主光轴与水平面的夹角
        logging.info("h: %f" % (h) + "______________Camera_angle: %f" % (Camera_angle))
        logging.info("Head_angle: %f" % Head_angle)

    else:
        h = 0.57
        Camera_angle = 38  # 主光轴与水平面的夹角
    logging.info("h: %f" % h)
    logging.info("camera_angle: %f" % Camera_angle)

    Total_angle = math.pi * (Picture_angle + Camera_angle) / 180 + Head_angle
    logging.info("Totl_angle %f" % Total_angle)

    d1 = h / math.tan(Total_angle)
    logging.info("d1: %f" % d1)

    # alpha = math.pi * (distx * 60.89 / 320) / 180  # 小球与主光轴的水平偏角
    alpha = math.pi * (distx * 60.92 / 320) / 180  # 小球与主光轴的水平偏角
    d2 = d1 / math.cos(alpha)
    d2 = d2 * 100
    logging.info("水平视角= 60.92")
    logging.info("angle= %f , distance= %f" % (alpha * almath.TO_DEG, d2))
    return d2, alpha


# ----------------------------Landmark相关---------------------------------
def searchLandmarkInRight():
    """
    搜索Landmark，返回
    :return:landmarkData(角度，距离),isFindLandMark
    """
    global g_motion, g_landmarkDetection, g_camera
    isFindLandmark = False  # landmark识别符0代表识别到，1代表未识别到。
    robotToLandmarkData = []
    headYawAngle = -110 * almath.TO_RAD  # 摆头角度，从右往左扫
    currentCamera = "CameraTop"
    # 初始化
    # # 重置头部角度
    # g_motion.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.3)
    # g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.3)
    # 设置刚度为 1 保证其头部能够运转
    g_motion.setStiffnesses("Head", 1.0)
    # 开启上摄像头
    g_camera.setActiveCamera(0)
    # 注册事件
    g_landmarkDetection.subscribe("landmarkTest")

    g_motion.angleInterpolationWithSpeed("HeadPitch", -0.1, 0.2)
    # time.sleep(2)
    # 初始为-1.5，及从由往左观察
    times = 0
    addAngle = 39 * almath.TO_RAD
    while times < 2:
        time.sleep(3)
        markData = g_memory.getData("LandmarkDetected")
        # 找到landmark
        if (markData and isinstance(markData, list) and len(markData) >= 2):
            # 提示
            g_tts.say("find landmark!")
            # 置标志为rue
            isFindLandmark = True  # landmark识别符1代表识别到，0代表未识别到。
            # Retrieve landmark center position in radians.
            # 获取数据
            alpha = markData[1][0][0][1]
            beta = markData[1][0][0][2]
            # Retrieve landmark angular size in radians.
            landmarkWidthInPic = markData[1][0][0][3]
            # logging.info(("alpha: %f" %alpha))
            # logging.info(("beta: %f" % beta))
            # logging.info(landmarkWidthInPic)

            # 获取头转动角度
            headAngle = g_motion.getAngles("HeadYaw", True)

            logging.info(("headAngle: %f" % headAngle[0]))
            markWithHeadAngle = alpha + headAngle[0]  # landmark相对机器人头的角度

            # 头部正对landmark
            g_motion.angleInterpolationWithSpeed("HeadYaw", markWithHeadAngle, 0.2)

            # ----------------------------------计算距离-----------------------------------------------#
            distanceFromCameraToLandmark = landmark["size"] / (2 * math.tan(landmarkWidthInPic / 2))

            # 获取当前机器人到摄像头的距离的变换矩阵
            transform = g_motion.getTransform(currentCamera, 2, True)
            transformList = almath.vectorFloat(transform)
            robotToCamera = almath.Transform(transformList)
            # 打印
            # logging.info("transform:%f" % transform)
            # logging.info("transformList :%f" % transformList)
            # logging.info("robotToCamera :%f" % robotToCamera)

            # 计算指向landmark的旋转矩阵
            cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, beta, alpha)

            # 摄像头到landmark的矩阵
            cameraToLandmarkTranslationTransform = almath.Transform(distanceFromCameraToLandmark, 0, 0)

            # 机器人到landmark的矩阵
            robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform * cameraToLandmarkTranslationTransform
            # 打印
            # logging.info("cameraToLandmarkRotationTransform: " ,cameraToLandmarkRotationTransform)
            # logging.info("cameraToLandmarkTranslationTransform: %f" %cameraToLandmarkTranslationTransform)
            # logging.info("robotToLandmark%f" %robotToLandmark )
            x = robotToLandmark.r1_c4
            y = robotToLandmark.r2_c4
            z = robotToLandmark.r3_c4

            distance = math.sqrt(x ** 2 + y * y) * 100

            markWithHeadAngle = round(markWithHeadAngle, 2)
            # 将数据存入列表
            robotToLandmarkData.append(distance)
            robotToLandmarkData.append(markWithHeadAngle)
            # 记录日志
            logging.info("x = " + str(x))
            logging.info("y = " + str(y))
            logging.info("z = " + str(z))
            logging.info("nao与landmark的距离 :: " + str(distance) + "   角度:: " + str(markWithHeadAngle * almath.TO_DEG))
            # 找到landmark，跳出循环
            break

        # 没找到landmark，该变角度继续扫视
        g_motion.angleInterpolationWithSpeed("HeadYaw", headYawAngle, 0.4)
        if headYawAngle * almath.TO_DEG > 130:
            headYawAngle = -130 * almath.TO_RAD  # 摆头角度，从右往左扫
            times += 1
            continue
        # elif times == 0:
        #     headYawAngle = headYawAngle + (55 * almath.TO_RAD)
        # elif times == 1:
        headYawAngle = headYawAngle + (39 * almath.TO_RAD)
        # if headYawAngle
        # else:
        #     headYawAngle = headYawAngle - (20 * almath.TO_RAD)
    # 提示
    if not isFindLandmark:
        logging.info("landmark is not in sight !")
        g_tts.say("landmark is not in sight")
    # 取消事件
    g_landmarkDetection.unsubscribe("landmarkTest")
    g_camera.unsubscribe("AL::kTopCamera")

    # 调整头的角度
    g_motion.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.2)
    g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.2)
    return robotToLandmarkData, isFindLandmark


def searchLandmarkInLeft():
    """
    搜索Landmark，返回
    :return:landmarkData(角度，距离),isFindLandMark
    """
    global g_motion, g_landmarkDetection, g_camera
    isFindLandmark = False  # landmark识别符0代表识别到，1代表未识别到。
    robotToLandmarkData = []
    headYawAngle = 110 * almath.TO_RAD  # 摆头角度，从右往左扫
    currentCamera = "CameraTop"
    # 初始化
    # # 重置头部角度
    # g_motion.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.3)
    # g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.3)
    # 设置刚度为 1 保证其头部能够运转
    g_motion.setStiffnesses("Head", 1.0)
    # 开启上摄像头
    g_camera.setActiveCamera(0)
    # 注册事件
    g_landmarkDetection.subscribe("landmarkTest")

    g_motion.angleInterpolationWithSpeed("HeadPitch", -0.1, 0.2)
    # time.sleep(2)
    # 初始为-1.5，及从由往左观察
    times = 0
    addAngle = -39 * almath.TO_RAD
    while times < 2:
        time.sleep(3)
        markData = g_memory.getData("LandmarkDetected")
        # 找到landmark
        if (markData and isinstance(markData, list) and len(markData) >= 2):
            # 提示
            g_tts.say("find landmark!")
            # 置标志为rue
            isFindLandmark = True  # landmark识别符1代表识别到，0代表未识别到。
            # Retrieve landmark center position in radians.
            # 获取数据
            alpha = markData[1][0][0][1]
            beta = markData[1][0][0][2]
            # Retrieve landmark angular size in radians.
            landmarkWidthInPic = markData[1][0][0][3]
            # logging.info(("alpha: %f" %alpha))
            # logging.info(("beta: %f" % beta))
            # logging.info(landmarkWidthInPic)

            # 获取头转动角度
            headAngle = g_motion.getAngles("HeadYaw", True)

            logging.info(("headAngle:%f" % headAngle[0]))
            markWithHeadAngle = alpha + headAngle[0]  # landmark相对机器人头的角度

            # 头部正对landmark
            g_motion.angleInterpolationWithSpeed("HeadYaw", markWithHeadAngle, 0.2)

            # ----------------------------------计算距离-----------------------------------------------#
            distanceFromCameraToLandmark = landmark["size"] / (2 * math.tan(landmarkWidthInPic / 2))

            # 获取当前机器人到摄像头的距离的变换矩阵
            transform = g_motion.getTransform(currentCamera, 2, True)
            transformList = almath.vectorFloat(transform)
            robotToCamera = almath.Transform(transformList)
            # 打印
            # logging.info("transform:%f" % transform)
            # logging.info("transformList :%f" % transformList)
            # logging.info("robotToCamera :%f" % robotToCamera)

            # 计算指向landmark的旋转矩阵
            cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, beta, alpha)

            # 摄像头到landmark的矩阵
            cameraToLandmarkTranslationTransform = almath.Transform(distanceFromCameraToLandmark, 0, 0)

            # 机器人到landmark的矩阵
            robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform * cameraToLandmarkTranslationTransform
            # 打印
            # logging.info("cameraToLandmarkRotationTransform: " ,cameraToLandmarkRotationTransform)
            # logging.info("cameraToLandmarkTranslationTransform: %f" %cameraToLandmarkTranslationTransform)
            # logging.info("robotToLandmark%f" %robotToLandmark )
            x = robotToLandmark.r1_c4
            y = robotToLandmark.r2_c4
            z = robotToLandmark.r3_c4

            distance = math.sqrt(x ** 2 + y * y) * 100  # 机器人与mark的距离distance

            markWithHeadAngle = round(markWithHeadAngle, 2)
            # 将数据存入列表
            robotToLandmarkData.append(distance)
            robotToLandmarkData.append(markWithHeadAngle)
            # 记录日志
            logging.info("x = " + str(x))
            logging.info("y = " + str(y))
            logging.info("z = " + str(z))
            logging.info("nao与landmark的距离 :: " + str(distance) + "   角度:: " + str(markWithHeadAngle * almath.TO_DEG))
            # 找到landmark，跳出循环
            break

        # 没找到landmark，该变角度继续扫视
        g_motion.angleInterpolationWithSpeed("HeadYaw", headYawAngle, 0.4)
        if headYawAngle * almath.TO_DEG < -130:
            headYawAngle = 130 * almath.TO_RAD  # 摆头角度，从右往左扫
            times += 1
            continue
        # elif times == 0:
        #     headYawAngle = headYawAngle + (55 * almath.TO_RAD)
        # elif times == 1:
        headYawAngle = headYawAngle + addAngle
        # if headYawAngle
        # else:
        #     headYawAngle = headYawAngle - (20 * almath.TO_RAD)
    # 提示
    if not isFindLandmark:
        logging.info("landmark is not in sight !")
        g_tts.say("landmark is not in sight")
    # 取消事件
    g_landmarkDetection.unsubscribe("landmarkTest")
    g_camera.unsubscribe("AL::kTopCamera")
    g_camera.closeCamera(0)
    # 调整头的角度
    g_motion.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.2)
    g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.2)
    return robotToLandmarkData, isFindLandmark


def searchLandmarkForAvoid():
    """
    搜索Landmark，返回
    :return:landmarkData(角度，距离),isFindLandMark
    """
    global g_motion, g_landmarkDetection, g_camera
    isFindLandmark = False  # landmark识别符0代表识别到，1代表未识别到。
    robotToLandmarkData = []
    headYawAngle = 110 * almath.TO_RAD  # 摆头角度，从右往左扫
    currentCamera = "CameraTop"
    # 初始化
    # # 重置头部角度
    # g_motion.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.3)
    # g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.3)
    # 设置刚度为 1 保证其头部能够运转
    g_motion.setStiffnesses("Head", 1.0)
    # 开启上摄像头
    g_camera.setActiveCamera(0)
    # 注册事件
    g_landmarkDetection.subscribe("landmarkTest")

    g_motion.angleInterpolationWithSpeed("HeadPitch", -0.1, 0.3)
    # time.sleep(2)
    # 初始为-1.5，及从由往左观察
    times = 0
    addAngle = -39 * almath.TO_RAD
    time.sleep(2.5)
    markData = g_memory.getData("LandmarkDetected")
    # 找到landmark
    if (markData and isinstance(markData, list) and len(markData) >= 2):
        # 提示
        g_tts.say("find landmark!")
        # 置标志为rue
        isFindLandmark = True  # landmark识别符1代表识别到，0代表未识别到。
        # Retrieve landmark center position in radians.
        # 获取数据
        alpha = markData[1][0][0][1]
        beta = markData[1][0][0][2]
        # Retrieve landmark angular size in radians.
        landmarkWidthInPic = markData[1][0][0][3]
        # logging.info(("alpha: %f" %alpha))
        # logging.info(("beta: %f" % beta))
        # logging.info(landmarkWidthInPic)

        # 获取头转动角度
        headAngle = g_motion.getAngles("HeadYaw", True)

        logging.info(("headAngle:%f" % headAngle[0]))
        markWithHeadAngle = alpha + headAngle[0]  # landmark相对机器人头的角度

        # 头部正对landmark
        g_motion.angleInterpolationWithSpeed("HeadYaw", markWithHeadAngle, 0.2)

        # ----------------------------------计算距离-----------------------------------------------#
        distanceFromCameraToLandmark = landmark["size"] / (2 * math.tan(landmarkWidthInPic / 2))

        # 获取当前机器人到摄像头的距离的变换矩阵
        transform = g_motion.getTransform(currentCamera, 2, True)
        transformList = almath.vectorFloat(transform)
        robotToCamera = almath.Transform(transformList)
        # 打印
        # logging.info("transform:%f" % transform)
        # logging.info("transformList :%f" % transformList)
        # logging.info("robotToCamera :%f" % robotToCamera)

        # 计算指向landmark的旋转矩阵
        cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, beta, alpha)

        # 摄像头到landmark的矩阵
        cameraToLandmarkTranslationTransform = almath.Transform(distanceFromCameraToLandmark, 0, 0)

        # 机器人到landmark的矩阵
        robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform * cameraToLandmarkTranslationTransform
        # 打印
        # logging.info("cameraToLandmarkRotationTransform: " ,cameraToLandmarkRotationTransform)
        # logging.info("cameraToLandmarkTranslationTransform: %f" %cameraToLandmarkTranslationTransform)
        # logging.info("robotToLandmark%f" %robotToLandmark )
        x = robotToLandmark.r1_c4
        y = robotToLandmark.r2_c4
        z = robotToLandmark.r3_c4

        distance = math.sqrt(x ** 2 + y * y) * 100  # 机器人与mark的距离distance
        # distance = math.sqrt((x+0.055)**2 + y * y) * 100  # 机器人与mark的距离distance
        # # 一些修正处理
        # x*=100
        # y*=100
        # l  = math.sqrt(x**2 + y * y)
        # modifyAngle =  math.asin((x+5.5)/distance) - math.asin(x/l)
        # logging.debug("modifyAngle:::::" + str(modifyAngle * almath.TO_DEG))
        # if markWithHeadAngle > 0:
        #     markWithHeadAngle-=modifyAngle
        # else:
        #     markWithHeadAngle+=modifyAngle
        # 修正值
        # distance = round( getRealDistanceForLandmark(distance, int(markWithHeadAngle * almath.TO_DEG) ), 2)

        markWithHeadAngle = round(markWithHeadAngle, 2)
        # 将数据存入列表
        robotToLandmarkData.append(distance)
        robotToLandmarkData.append(markWithHeadAngle)
        # 记录日志
        logging.info("x = " + str(x))
        logging.info("y = " + str(y))
        logging.info("z = " + str(z))
        logging.info("nao与landmark的距离 :: " + str(distance) + "   角度:: " + str(markWithHeadAngle * almath.TO_DEG))
        # 找到landmark，跳出循环

    # 提示
    if not isFindLandmark:
        logging.info("landmark is not in sight !")
        g_tts.say("landmark is not in sight")
    # 取消事件
    g_landmarkDetection.unsubscribe("landmarkTest")
    g_camera.closeCamera(0)
    # 调整头的角度
    g_motion.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.2)
    g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.2)
    return robotToLandmarkData, isFindLandmark


# -------------找到球的一些行为--------------
# def FirstFindBall(distance, headYawAngle):
#     '''
#     找到球后，正对球，调整距离
#     :param distance: NAO与球的距离
#     :param headYawAngle: NAO与球的角度
#     :return:
#     '''
#     logging.info("-----------------------FirstFindBall----------------")
#     # 正对
#     move(theta=headYawAngle)
#     # 计算距离
#     distance, Angle, ballCoord = trackBall(headang)
#     times = 0
#     # 防止找球不精确
#     while abs(ballCoord[2]) > 60:
#         distance, Angle, ballCoord = trackBall(headang)
#         if times > 5:
#             g_tts.say("ball data error")
#             break
#         times += 1
#     moveDistance = distance - 15
#     # 移动到指定距离
#     if moveDistance > 40:
#         move(30)
#         distance, Angle, ballCoord = trackBall(headang)
#         FirstFindBall(distance, Angle)
#     else:
#         move(x=moveDistance)
#         # 再次正对
#         time.sleep(0.5)
#         distance, Angle, ballCoord = trackBall(headang)
#         times = 0
#         # 防止找球不精确
#         while abs(ballCoord[2]) > 50:
#             distance, Angle, ballCoord = trackBall(headang)
#             if times > 5:
#                 g_tts.say("ball data error")
#                 break
#             times += 1
#         move(theta=Angle)  # 正对
#         # 一些意外情况处理
#         if distance < 13 or distance > 17:
#             move(distance - 15)
def actionAfterFirstFindBall(distance, headYawAngle):
    '''
    找到球后，正对球，调整距离
    :param distance: NAO与球的距离
    :param headYawAngle: NAO与球的角度
    :return:
    '''
    logging.info("-----------------------actionAfterFirstFindBall----------------")

    # 计算距离
    # 正对
    move(theta=headYawAngle)
    distance, Angle, ballCoord = trackBall(headang)
    times = 0
    # 防止找球不精确
    while abs(ballCoord[2]) > 50:
        distance, Angle, ballCoord = trackBall(headang)
        if times > 5:
            g_tts.say("ball data error,I can't continue")
            break
        times += 1
    moveDistance = distance - 15
    # 移动到指定距离
    if moveDistance > 40:
        move(30)
        distance, Angle, ballCoord = trackBall(headang)
        actionAfterFirstFindBall(distance, Angle)
    else:
        move(x=moveDistance)
        # 再次正对
        time.sleep(0.5)
        distance, Angle, ballCoord = trackBall(headang)
        times = 0
        # 防止找球不精确
        while abs(ballCoord[2]) > 50:
            distance, Angle, ballCoord = trackBall(headang)
            if times > 5:
                g_tts.say("ball data error,I can't continue")
                break
            times += 1
        move(theta=Angle)  # 正对
        # 一些意外情况处理
        if distance < 13 or distance > 17:
            distance, Angle, ballCoord = trackBall(headang)
            times = 0
            # 防止找球不精确
            while abs(ballCoord[2]) > 50:
                distance, Angle, ballCoord = trackBall(headang)
                if times > 5:
                    g_tts.say("ball data error,I can't continue")
                    break
                times += 1
            move(distance - 15)


def firstClosingForHitBall(angleForLandMark):
    """
    根据与landmark的角度的正负决定击球方式，先确定击球点在进行调整
    :param angleForLandMark:
    :return:
    """
    # 1、重新获取距离
    # distance2Ball, headYawAngle, ballCoord = trackBall(headang)
    # move(theta=headYawAngle)
    logging.info("-----------------------firstClosingForHitBall----------------")
    distance2Ball, headYawAngle, ballCoord = trackBall(headang)
    times = 0
    while abs(ballCoord[2]) > 50:
        distance2Ball, headYawAngle, ballCoord = trackBall(headang)
        if times > 5:
            g_tts.say("ball data error,I can't continue")
            stop()
            exit(1)
        times += 1
    if distance2Ball > 10:
        move(x=distance2Ball-10)
    distance2Ball, headYawAngle, ballCoord = trackBall(headang)
    times = 0
    while abs(ballCoord[2]) > 50:
        distance2Ball, headYawAngle, ballCoord = trackBall(headang)
        if times > 5:
            g_tts.say("ball data error,I can't continue")
            stop()
            exit(2)
        times += 1
    # 2、判断机器人在球朝向Landmark的线上的哪边
    # 2.1、在左边，及angleForLandMark值为正
    # 调整距离与角度
    adjustY = ballCoord[1] - 4
    move1(y=adjustY)  # 调到左脚前方
    # distance2Ball, headYawAngle, ballCoord = trackBall(headang)
    # while abs(ballCoord[2]) > 50:
    #     distance2Ball, headYawAngle, ballCoord = trackBall(headang)
    #
    # if ballCoord[1] > 4 or ballCoord[1] < 2:
    #     adjustY = ballCoord[1] - 4
    #     move1(y=adjustY)  # 调到左脚前方
    #
    # distance2Ball, headYawAngle, ballCoord = trackBall(headang)
    # times = 0
    # while abs(ballCoord[2]) > 40:
    #     distance2Ball, headYawAngle, ballCoord = trackBall(headang)
    #     if times > 5:
    #         g_tts.say("ball data error,I can't continue")
    #         stop()
    #         exit(2)
    #     times += 1
    #
    # adjustX = distance2Ball - 9  # 调到左脚前方距离为9cm
    # move(x=adjustX)

    if angleForLandMark > 0:  # 如果landmark在左边
        # 微调
        for t in range(2):
            logging.info("landmark在左微调")
            # 微调y
            # 重新获取与球相关的值
            distance2Ball, headYawAngle, ballCoord = trackBallNoHead(headang)
            logging.info('ballCoord[1] %f' % ballCoord[1])
            while (distance2Ball > 15 or ballCoord[2] > 30):
                distance2Ball, headYawAngle, ballCoord = trackBallNoHead(headang)

            if (3 < ballCoord[1] < 6):
                if 7.8 < distance2Ball < 9.5:
                    #distance2Ball, headYawAngle, ballCoord = trackBallNoHead(headang)
                    if 3.5 < ballCoord[1] < 5:
                        move1(y=-2)
                        break
                    else:
                        move1(y=ballCoord[1] - 6)
                        break
                else:
                    logging.info("distance2Ball > 9.5 or distance2Ball < 7.8")
                    move(x=distance2Ball - 9)
                    if t==0:
                        move1(y=-2)
                    elif t==1:
                        move1(y=-1)
            elif ballCoord[1] <= 3:
                logging.info("ballCoord[1] <=3")
                move1(y=-4)
            elif ballCoord[1] >= 8:
                logging.info("ballCoord[1] > 8")
                move1(y=2)
            elif 8 > ballCoord[1] >= 7:
                logging.info("ballCoord[1] >=7")
                move1(y=1)
        # 判断？
        # 击球
        pass
    else:  # 如果landmark在右边
        for t in range(1):
            logging.info("landmark在右微调")
            # 微调y
            # 重新获取与球相关的值
            distance2Ball, headYawAngle, ballCoord = trackBallNoHead(headang)
            logging.info('ballCoord[1] %f' % ballCoord[1])
            while (distance2Ball > 15 or ballCoord[2] > 30):
                distance2Ball, headYawAngle, ballCoord = trackBallNoHead(headang)
            if (5 <= ballCoord[1] < 7) or (t % 2 == 1):
                if 10 < distance2Ball < 12:
                    #move(y=-2)
                    break

                else:
                    logging.info("distance2Ball > 9.5 or distance2Ball < 7.8")
                    move(x=distance2Ball - 11)
                    #move(y=-1)  # 非本地碰球
            elif 4 <= ballCoord[1] <5:
                logging.info("ballCoord[1] < 5")
                move1(y=-2)
            elif 2 < ballCoord[1] <4:
                logging.info("ballCoord[1] < 4")
                move1(y=-3)
            elif ballCoord[1] <= 2:
                logging.info("ballCoord[1] <= 2")
                move1(y=-4)
            elif ballCoord[1] >= 7:
                logging.info("ballCoord[1] >= 7")
                move1(y=2)


# -------------一---------------------些计算---------------------------------
# 利用了一些简单的三角函数
def calDistanceFromBall2Mark(distance2Ball, distance2Mark, angleForLandMark):
    # type: (float, float, float) -> float
    """
    根据nao与球的距离，nao与landmark的距离以及角度，计算球和landmark的距离
    :param distance2Mark: ao与球的距离 cm
    :param angleForLandMark: nao与landmark的角度 弧度值表示
    :param distance2Ball: nao与landmark的距离 cm
    :return:球和landmark的距离
    """
    logging.debug("--------------------------calDistanceFromBall2Mark---------------------")
    logging.info("distance2Ball:: " + str(distance2Ball))
    logging.info("distance2Mark:: " + str(distance2Mark))
    logging.info("angleForLandMark:: " + str(angleForLandMark))
    distanceFromBall2Mark = math.sqrt(
        distance2Ball ** 2 + distance2Mark ** 2 - 2 * distance2Mark * distance2Ball * (math.cos(abs(angleForLandMark))))
    logging.info("distanceFromBall2Mark-->" + str(distanceFromBall2Mark))
    return round(distanceFromBall2Mark, 2)


def calAdjustY(distance2Ball, distance2Mark, angleForLandMark, distanceFromBall2Mark):
    """
    根据nao与球的距离，nao与landmark的距离以及角度，计算形成直角需要修正的Y值
    :param distance2Ball: nao与landmark的距离 cm
    :param distance2Mark: ao与球的距离 cm
    :param angleForLandMark: nao与landmark的角度 弧度值表示
    :return: 需要修正的Y值
    """
    logging.debug("--------------------------calAdjustY---------------------")
    angleForBall = math.acos((distance2Ball - distance2Mark * math.cos(abs(angleForLandMark))) / distanceFromBall2Mark)
    logging.debug("angleForBall ---> " + str(angleForBall))
    distanceY = math.tan(abs(angleForBall - math.pi / 2)) * distance2Ball
    adjustTheta = -abs(math.pi / 2 - angleForBall)
    if angleForLandMark > 0 and (angleForBall - math.pi / 2) < 0:  # 球在landmark右边并且过了球洞
        distanceY = -distanceY
        adjustTheta = -adjustTheta
    elif angleForLandMark < 0 and (angleForBall - math.pi / 2) > 0:  # 球在landmark左边并且未过球洞
        distanceY = -distanceY
        adjustTheta = -adjustTheta
    # elif angleForLandMark > 0 and (angleForBall - math.pi / 2) > 0:#球在landmark右边并且未过球洞
    #     distanceY = distanceY
    #     adjustTheta = adjustTheta
    # elif angleForLandMark < 0 and (angleForBall - math.pi / 2) < 0:#球在landmark左边并且过了球洞
    #     distanceY = distanceY
    #     adjustTheta = adjustTheta
    logging.info("distanceY-->" + str(distanceY) + "  adjustTheta-->" + str(adjustTheta * almath.TO_DEG))
    return round(distanceY, 2), round(adjustTheta * almath.TO_DEG, 1)


# ----------------------找球并击球---------------------------
def findAndHitBall(changci=1):
    distanceFromBall2Mark = None
    robotToLandmarkData = []
    logging.info("--------------findAndHitBall----------------")
    distance, headYawAngle, ballCoord = trackBall(headang)
    # 此处经修改后暂无意义
    for times in xrange(1):
        # 4.1、找到球
        if distance != -1:
            # 5、调整与球之间的距离
            actionAfterFirstFindBall(distance, headYawAngle)
            logging.info("Before find mark")
            # 6、找Landmark
            distanceToLandmark = 0
            if changci != 2:
                if headYawAngle < 0:
                    robotToLandmarkData, isFind = searchLandmarkInLeft()
                else:
                    robotToLandmarkData, isFind = searchLandmarkInRight()
            else:
                robotToLandmarkData, isFind = searchLandmarkInRight()
            # 没找到，继续扫
            if not isFind:
                if changci != 2:
                    if headYawAngle < 0:
                        robotToLandmarkData, isFind = searchLandmarkInRight()
                    else:
                        robotToLandmarkData, isFind = searchLandmarkInLeft()
                else:
                    robotToLandmarkData, isFind = searchLandmarkInLeft()
            # 6.1、找到landmark
            if isFind:
                logging.debug("NAO与landMark的距离:  " + str(robotToLandmarkData[0])
                              + "  角度:  " + str(robotToLandmarkData[1]))
                # 7、调整角度
                # 7.1 重新计算与球的距离,并获取球与landmark的距离
                distance2Ball, headYawAngle, ballCoord = trackBall(headang)
                # 防止数据异常
                for i in range(5):
                    if abs(ballCoord[2]) > 50:
                        distance2Ball, headYawAngle, ballCoord = trackBall(headang)
                    else:
                        break
                # 一些修正：根据实际情况进行调整
                # if headYawAngle < 0:
                # if robotToLandmarkData[1] > 0:#角度
                #     if distance2Ball < 13:
                #         distance2Ball=13
                #     distance2Ball +=  15 # 右边15
                #     # distance2Ball = 26 # 右边15
                #     # distance2Ball = 23 # 右边15
                #     if robotToLandmarkData[0] < 35:#距离
                #         distance2Ball-=2
                # else:
                #     # distance2Ball = 14.5
                #     if distance2Ball < 12:
                #         distance2Ball=12
                #     if robotToLandmarkData[0] < 35:
                #         distance2Ball+=1
                #     distance2Ball += 3.5 #左边
                #     # distance2Ball += 2.3 #左边

                distanceFromBall2Mark = calDistanceFromBall2Mark(distance2Ball, robotToLandmarkData[0],
                                                                 robotToLandmarkData[1])
                logging.info("distanceFromBall2Mark:  " + str(distanceFromBall2Mark))
                # 7.2、计算需移动的y值
                adjustY, adjustTheta = calAdjustY(distance2Ball, robotToLandmarkData[0], robotToLandmarkData[1],
                                                  distanceFromBall2Mark)
                # 7.3 调整距离和角度
                # time.sleep(0.5)
                # 若角度过大，进行二次修正
                if adjustY < 24 and adjustY > -24:
                    move1(y=adjustY)
                    if adjustTheta > 0:
                        if adjustTheta <= 5:
                            move(theta=adjustTheta - 1)#
                        elif 5 < adjustTheta <= 15:
                            move(theta=adjustTheta - 2) #   14
                        elif 15 < adjustTheta <= 22:
                            move(theta=adjustTheta - 2)  #  16
                        elif 22 < adjustTheta <= 28:
                            move(theta=adjustTheta-2)  #  23
                        elif 28 < adjustTheta <= 35:
                            move(theta=adjustTheta )  #  31
                        elif 35 < adjustTheta <= 38:
                            move(theta=adjustTheta+1)  #
                        elif 38 < adjustTheta <= 40:
                            move(theta=adjustTheta+1)  #
                        elif 40 < adjustTheta <= 50:
                            move(theta=adjustTheta + 2)  #  40，43
                        elif 50 < adjustTheta <= 60:
                            move(theta=adjustTheta + 2)  #
                        else:
                            move(theta=adjustTheta + 4)  # 12
                    else:
                        if 0 > adjustTheta >= -5:
                            move(theta=adjustTheta-1)  # -1
                        elif -5> adjustTheta >= -10:
                            move(theta=adjustTheta)
                        elif -10 > adjustTheta > -20:
                            move(theta=adjustTheta)  # √#-13
                        elif -20 >= adjustTheta > -30:
                            move(theta=adjustTheta)  # ,,√ -20
                        elif -30 >= adjustTheta > -40:
                            move(theta=adjustTheta + 1)  # √   ,,√
                        elif -40 >= adjustTheta > -50:
                            move(theta=adjustTheta + 2)  # √  -44
                        else:
                            move(theta=adjustTheta + 3)
                elif adjustY > 0:
                    if adjustTheta > 0:
                        move(theta=15)
                    else:
                        move(theta=-15)
                        # move(theta=adjustTheta / 1.5)
                    move(y=24)
                else:
                    if adjustTheta > 0:
                        move(theta=15)
                    else:
                        move(theta=-15)
                    move(y=-24)
                    # move(theta=adjustTheta / 1.5)
                # time.sleep(0.5)

                if adjustY > 24 or adjustY < -24:  # 25 -25
                    if changci == 2:
                        findAndHitBall()
                    else:
                        findAndHitBall2()
                else:
                    # 7.4 、调整至击球点
                    firstClosingForHitBall(robotToLandmarkData[1])
                    # 重新获取数据
                    # distance2Ball_2, headYawAngle_2, ballCoord_2 = trackBall(headang)

                    g_motion.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.1)
                    g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1)
                    # 7.5 击球
                    # 根据位置的不同决定击球方式
                    if distanceFromBall2Mark != None:
                        if robotToLandmarkData[1] > 0:
                            outShotBall(120)
                        elif robotToLandmarkData[1] < 0:
                            insideShotBall(120)
                            # 收杆
                    actionBeforeMove()
                    break
            # 判断击球次数
            # if times > 2:
            #     break

            # 没找到landmark
            else:
                pass
        # 没找到球
        else:
            return False
    return True


# 下面基本与第一个相同，都是根据实际情况做的一些调整
def findAndHitBall2(changci=1):
    distanceFromBall2Mark = None
    robotToLandmarkData = []
    logging.info("--------------findAndHitBall2----------------")
    distance, headYawAngle, ballCoord = trackBall(headang)
    for times in xrange(1):
        # 4.1、找到球
        if distance != -1:
            # 5、调整与球之间的距离
            FirstFindBall(distance, headYawAngle)
            logging.info("Before find mark")
            # 6、找Landmark
            distanceToLandmark = 0
            if changci != 2:
                if headYawAngle < 0:
                    robotToLandmarkData, isFind = searchLandmarkInLeft()
                else:
                    robotToLandmarkData, isFind = searchLandmarkInRight()
            else:
                robotToLandmarkData, isFind = searchLandmarkInRight()

            if isFind:
                logging.debug("NAO与landMark的距离:  " + str(robotToLandmarkData[0])
                              + "  角度:  " + str(robotToLandmarkData[1]))
                # 7、调整角度
                # 7.1 重新计算与球的距离,并获取球与landmark的距离
                distance2Ball, headYawAngle, ballCoord = trackBall(headang)
                # 防止数据异常
                for i in range(5):
                    if abs(ballCoord[2]) > 50:
                        distance2Ball, headYawAngle, ballCoord = trackBall(headang)
                    else:
                        break

                logging.info("adjustTwice")
                # distance2Ball += (15 - distance2Ball) + 13  # 右边15
                # if robotToLandmarkData[0] < 45:
                #     distance2Ball -= 2

                distanceFromBall2Mark = calDistanceFromBall2Mark(distance2Ball, robotToLandmarkData[0],
                                                                 robotToLandmarkData[1])
                logging.info("distanceFromBall2Mark:  " + str(distanceFromBall2Mark))
                # 7.2、计算需移动的y值
                adjustY, adjustTheta = calAdjustY(distance2Ball, robotToLandmarkData[0], robotToLandmarkData[1],
                                                  distanceFromBall2Mark)

                # 7.3 调整距离和角度
                # time.sleep(0.5)
                if adjustY < 28 and adjustY > -28:
                    move(y=adjustY)
                elif adjustY > 0:
                    move(y=28)
                else:
                    move(y=-28)
                # time.sleep(0.5)
                if adjustY < 24 and adjustY > -24:
                    move(y=adjustY)
                    if adjustTheta > 0:
                        if adjustTheta <= 5:
                            move(theta=adjustTheta - 1)#
                        elif 5 < adjustTheta <= 15:
                            move(theta=adjustTheta - 2) #   14
                        elif 15 < adjustTheta <= 22:
                            move(theta=adjustTheta - 2)  #  16
                        elif 22 < adjustTheta <= 28:
                            move(theta=adjustTheta-2)  #  23
                        elif 28 < adjustTheta <= 35:
                            move(theta=adjustTheta )  #  31
                        elif 35 < adjustTheta <= 38:
                            move(theta=adjustTheta+1)  #
                        elif 38 < adjustTheta <= 40:
                            move(theta=adjustTheta+1)  #
                        elif 40 < adjustTheta <= 50:
                            move(theta=adjustTheta + 2)  #  40，43
                        elif 50 < adjustTheta <= 60:
                            move(theta=adjustTheta + 2)  #
                        else:
                            move(theta=adjustTheta + 4)  # 12
                    else:
                        if 0 > adjustTheta >= -5:
                            move(theta=adjustTheta-1)  # -1
                        elif -5> adjustTheta >= -10:
                            move(theta=adjustTheta)
                        elif -10 > adjustTheta > -20:
                            move(theta=adjustTheta)  # √#-13
                        elif -20 >= adjustTheta > -30:
                            move(theta=adjustTheta)  # ,,√ -20
                        elif -30 >= adjustTheta > -40:
                            move(theta=adjustTheta + 1)  # √   ,,√
                        elif -40 >= adjustTheta > -50:
                            move(theta=adjustTheta + 2)  # √  -44
                        else:
                            move(theta=adjustTheta + 3)
                if adjustY > 24 or adjustY < -24:
                    findAndHitBall()
                else:
                    # 7.4 、调整至击球点
                    firstClosingForHitBall(robotToLandmarkData[1])

                    g_motion.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.1)
                    g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1)

                    # 7.5 击球
                    # 根据位置的不同决定击球方式
                    if distanceFromBall2Mark != None:
                        if robotToLandmarkData[1] > 0:
                            outShotBall(120)
                        elif robotToLandmarkData[1] < 0:
                            insideShotBall(120)
                            # 收杆
                    actionBeforeMove()
                    break
            # 判断击球次数
            # if times > 2:
            #     break

            # 没找到landmark
            else:
                pass
        # 没找到球
        else:
            return False
    return True


# ---------------------------击球相关-----------------------------
def insideShotBall(distance):
    # // 第二次击球的准备动作（内击）
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.4, 0.05)
    g_motion.angleInterpolationWithSpeed("LShoulderPitch", 0.5, 0.2)
    g_motion.angleInterpolationWithSpeed("LElbowYaw", -1.8, 0.2)
    g_motion.angleInterpolationWithSpeed("LElbowRoll", -0.83, 0.2)
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.55, 0.05)
    g_motion.angleInterpolationWithSpeed("LShoulderRoll", -1.7, 0.2)  # -1.5
    # time.sleep(3)
    g_motion.angleInterpolationWithSpeed("LShoulderPitch", 0.81, 0.05)
    # time.sleep(5)
    # // 击球（内击）
    g_motion.angleInterpolationWithSpeed("LWristYaw", -0.4, 0.28)
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.4, 0.05)

    # g_motion.angleInterpolati  g_motion.angleInterpolationWithSpeed("LWristYaw", 0.4, 0.05)
    #     g_motion.angleInterpolationWithSpeed("LShoulderPitch", 0.5, 0.2)
    #     g_motion.angleInterpolationWithSpeed("LElbowYaw", -1.8, 0.2)#负数手臂往外转
    #     g_motion.angleInterpolationWithSpeed("LElbowRoll", -0.83, 0.2)#肩膀的上下关节，负的越多越往上
    #     g_motion.angleInterpolationWithSpeed("LWristYaw", 0.6, 0.05)
    #     g_motion.angleInterpolationWithSpeed("LShoulderRoll", -1.5, 0.2)#肩膀最上面的关节，负的越多越往里
    #     g_motion.angleInterpolationWithSpeed("LShoulderPitch", 0.837, 0.05)onWithSpeed("LWristYaw", 0.4, 0.02)
    # g_motion.angleInterpolationWithSpeed("LShoulderPitch", 0.5, 0.02)
    # g_motion.angleInterpolationWithSpeed("LElbowYaw", -1.8, 0.02)
    # g_motion.angleInterpolationWithSpeed("LElbowRoll", -0.83, 0.02)
    # g_motion.angleInterpolationWithSpeed("LWristYaw", 0.6, 0.02)
    # g_motion.angleInterpolationWithSpeed("LShoulderRoll", -1.5, 0.02)
    # g_motion.angleInterpolationWithSpeed("LShoulderPitch", 0.837, 0.02)

    # time.sleep(4)

    # // 击球（内击）
    # g_motion.angleInterpolationWithSpeed("LWristYaw", -0.4, 0.12)
    # g_motion.angleInterpolationWithSpeed("LWristYaw", 0.4, 0.05)


def outShotBall(distance):
    # distance 球到naomark的距离,以厘米为单位

    # // 第二次击球的准备动作（外击）
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.1, 0.05)  # 0
    g_motion.angleInterpolationWithSpeed("LShoulderPitch", 0.19, 0.2)

    g_motion.angleInterpolationWithSpeed("LElbowRoll", -0.74, 0.2)  # -0.83
    g_motion.angleInterpolationWithSpeed("LShoulderRoll", -1.7, 0.2)  # -1.5
    g_motion.angleInterpolationWithSpeed("LElbowYaw", -1.8, 0.2)
    g_motion.angleInterpolationWithSpeed("LShoulderPitch", 0.75, 0.05)  # 0.837

    # // 击球（外击）
    g_motion.angleInterpolationWithSpeed("LWristYaw", -0.2, 0.1)  # <0往内
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.8, 0.1)  # 0.085#>0往外
    g_motion.angleInterpolationWithSpeed("LWristYaw", 0.4, 0.05)


# 停止函数，松杆并坐下
def stop(t=2):
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
    g_motion.rest()


def thirdMain():
    """第三场"""
    ## 1、初始化日志配置、加载模块、站立，接杆
    naoInit("NAOThree", IP)
    times = 0  # 除固定击球以外击球次数
    g_tts.say("three")
    # 第一次击球
    firstHitBallForThree()
    # 收杆
    actionBeforeMove()
    # 转向
    # 往前走一段
    # 找球，调整距离
    time.sleep(1)
    move(theta=-75)
    move2(x=100)
    # time.sleep(0.5)
    closingForHitBallForThree()
    # trackBall(headang)
    # move(y=-2)  # -3#-4#解决往左转8度，球y坐标会减少3的问题
    # move(theta=3)  # 5#8#往左转8度，与球的距离增大2，球y坐标减少3
    # move(x=2)  # 2#1

    # trackBall(headang)
    deal_theta()
    move(theta=0)
    distance0, headYawAngle0, ballCoord0 = trackBall(headang)
    print "距离:", distance0
    print "y:", ballCoord0[1]
    if distance0 >= 11.5:
        if 12.5 > distance0 >= 11.5:
            move(x=1)
        if 13.5 > distance0 >= 12.5:
            move(x=2)
        if 14.5 > distance0 >= 13.5:
            move(x=3)
        if distance0 >= 14.5:
            move(x=4)
    elif distance0 < 10.5:
        move(x=-1)
    secondHitBallForThree()
    # secondHitBallForThree1() # 特殊策略
    actionBeforeMove()

    move(theta=-68)  # -70#-67
    moveForThree(x=56 * 3.6)  # 4.5
    # move(theta=-5)

    # distance, headYawAngle, ballCoord = trackBallforthree(headang)
    # if distance > 0:
    #     findAndHitBall(changci=2)
    #     stop(1)
    #     exit(0)
    distance, angle = searchBallUseTop()
    g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1)  # 复原头
    if distance <= 0:  # 距离太近找不到用下摄像头，下摄像头没找到后退15，再用上摄像头 PS:如果距离太远找不到重开
        flag = findAndHitBall(changci=2)
        if not flag:
            for i in range(3):
                move(x=-15)
                distance, angle = searchBallUseTop()
                if distance > 0:
                    break
                g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1)  # 复原头
        else:
            stop(1)
            exit(0)
    # distance1 = distance1 * 100  # 将m化成cm
    logging.info("distance:%f" % distance)
    # 上摄像头找到球
    if distance > 48:
        move(theta=angle)
        move(distance - 25)
    flag1 = findAndHitBall(changci=2)
    if not flag1:
        for j in range(3):
            move(x=15)
            flag2 = findAndHitBall(changci=2)
            if flag2 == True:
                break
    stop(1)

    # distance1, angle1 = searchBallUseTop()
    # g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1)  # 复原头
    # for i in range(3):
    #     if distance1 <= 0:  # 如果距离太近找不到，后退15
    #         move(x=-15)
    #         distance1, angle1 = searchBallUseTop()
    #         g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1)  # 复原头
    #     else:
    #         break
    # # distance1 = distance1 * 100  # 将m化成cm
    # logging.info("distance1:%f" % distance1)
    # # 上摄像头找到球
    # if distance1 > 45:
    #     move(theta=angle1)
    #     move(distance1 - 25)
    # flag = findAndHitBall(changci=2)
    # if not flag:
    #     g_tts.say("now I use top to find ball")
    #     distance, angle = searchBallUseTop()
    #     g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1)  # 复原头
    #     for i in range(3):
    #         if distance <= 0:  # 如果距离太近找不到，后退15
    #             move(x=-15)
    #             distance, angle = searchBallUseTop()
    #             g_motion.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.1)  # 复原头
    #         else:
    #             break
    #     # distance = distance * 100  # 将m化成cm
    #     logging.info("distance2:%f" % distance)
    #     # 上摄像头找到球
    #     if distance > 0:
    #         move(theta=angle)
    #         move(distance - 25)
    #         findAndHitBall(changci=2)
    # stop(1)


thirdMain()
