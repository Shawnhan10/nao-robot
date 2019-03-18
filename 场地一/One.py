# -*- coding:utf-8 -*-
from naoqi import ALProxy
import time
# IP及端口
IP = "192.168.43.207"
# IP = "127.0.0.1"
Port = 9559

g_motion = ALProxy("ALMotion", IP, Port)  # 移动模块
g_posture = ALProxy("ALRobotPosture", IP, Port)  # 姿势模块
g_memory = ALProxy("ALMemory", IP, Port)  # 内存管理模块
# ---------------------------击球相关-----------------------------
g_posture.goToPosture("StandInit", 0.5)
g_motion.angleInterpolationWithSpeed("LWristYaw", 0.3, 0.2)
g_motion.angleInterpolationWithSpeed("LShoulderPitch", 0.77, 0.2)
g_motion.angleInterpolationWithSpeed("LShoulderRoll", -1.5, 0.2)
g_motion.angleInterpolationWithSpeed("LElbowRoll", -0.9, 0.2)
g_motion.angleInterpolationWithSpeed("LElbowYaw", -1.8, 0.2)
# 拿杆
g_motion.angleInterpolationWithSpeed("LHand", 1.0, 0.2)
time.sleep(5)
g_motion.angleInterpolationWithSpeed("LHand", 0.15, 0.2)
g_motion.angleInterpolationWithSpeed("RWristYaw", 0.6, 0.2)
g_motion.setStiffnesses("LHand", 1.0)

time.sleep(2)
g_motion.angleInterpolationWithSpeed("LWristYaw", 0.6, 0.26)#0.6

# def firstHitBallForOne2():
#     '''
#         第一场第一次击球
#     '''
#     global g_motion, g_memory
#     g_motion.angleInterpolationWithSpeed("LWristYaw", 1.2, 0.1)
#     g_motion.angleInterpolationWithSpeed("LWristYaw", -0.9, 0.55)#0.5 ,,0.6# 0.55第一
#     g_motion.angleInterpolationWithSpeed("LWristYaw", 0.4, 0.05)
def firstHitBall():
    g_motion.angleInterpolationWithSpeed("LWristYaw", -0.3, 0.29)#0.19
while True:
    if g_memory.getData("HandRightRightTouched"):#如果右手被触摸
        #firstHitBallForOne2()
        firstHitBall()
        time.sleep(1)
        g_motion.openHand("LHand")
        break