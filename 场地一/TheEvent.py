# -*- coding:UTF-8 -*-


import time
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
import os

# 后台运行需要命令
# ./pso > pso.file 2>&1 &
# nohup  ./pso > pso.file 2>&1 &
# nohup python TheEvent.py  > temp.log 2>&1 &
memory = None
# IP1 = "172.16.55.83"
IP = "127.0.0.1"

class ReactToTouch(ALModule):
    '''
     A simple module able to react to touch events
    '''
    def __init__(self, name):
        ALModule.__init__(self, name)
        # No need for IP and Port here because we have our Python broker connected to NAOqi broker
        # Create a proxy to ALTextToSpeech for later use
        self.tts = ALProxy("ALTextToSpeech")
        self.posture = ALProxy("ALRobotPosture")
        self.motion = ALProxy("ALMotion")
        self.tracker = ALProxy("ALTracker")
        self.landmarkDetected = ALProxy("ALLandMarkDetection")
        # Subscribe to TouchChange event:
        global memory
        memory = ALProxy("ALMemory")
        memory.subscribeToEvent("TouchChanged", "ReactToTouch", "onTouched")

    def onTouched(self, strVarName, value):
        '''
        This will be called each time a touch is detected
        :param strVarName:
        :param value:
        :return:
        '''
        # Unsubscribe to the event when talking, to avoid repetitions
        # memory.unsubscribeToEvent("TouchChanged", "ReactToTouch")

        print value
        if len(value) > 1:
            self.do(value[1])

        # Subscribe again to the event
        # memory.subscribeToEvent("TouchChanged", "ReactToTouch", "onTouched")

    def do(self, bodies):
        if bodies[1] == True and bodies[2] == [1]:
           if  self.motion.robotIsWakeUp():
               print "wakeUp"
               if bodies.count("Head/Touch/Front") > 0 :
                    self.killTask("One.py")
                    self.killTask("Two.py")
                    self.killTask("Three.py")
                    #self.killTask("find_shut.py")
                    #self.killTask("TheFirst_beiyong.py")
                    self.stop()
               elif bodies.count("Head/Touch/Middle") > 0:
                    self.killTask("One.py")
                    self.killTask("Two.py")
                    self.killTask("Three.py")
                    #self.killTask("find_shut.py")
                    #self.killTask("TheFirst_beiyong.py")
                    self.stop()
               elif bodies.count("Head/Touch/Rear") > 0:
                    self.killTask("One.py")
                    self.killTask("Two.py")
                    self.killTask("Three.py")
                    #self.killTask("find_shut.py")
                    #self.killTask("TheFirst_beiyong.py")
                    self.stop()
           else:
               print "not  wakeUp"
               if bodies.count("Head/Touch/Front") and bodies[1] == True > 0:
                   # if bodies.count("LHand/Touch/Left") and bodies[1] == True:
                   #     pass
                   os.system("python One.py")
                   #os.system("python find_shut.py")
               elif bodies.count("Head/Touch/Middle") and bodies[1] == True > 0:
                   # if bodies.count("Head/Touch/Front") and bodies[1] == True:
                   #     pass
                   os.system("python Two.py")
                   pass
               elif bodies.count("Head/Touch/Rear") and bodies[1] == True > 0:
                   os.system("python Three.py")
               #elif bodies.count("LHand/Touch/Back") and bodies[1] == True > 0:
                 #  os.system("python TheFirst_beiyong.py")
    def stop(self):
        self.motion.stopMove()
        # 停止追踪
        self.tracker.stopTracker()
        self.tracker.unregisterTarget("RedBall")
        self.tracker.unregisterAllTargets()
        landmarkInfo = self.landmarkDetected.getSubscribersInfo()
        try:
            for info in landmarkInfo:
                self.landmarkDetected.unsubscribe(info[0])
        except Exception, e:
            pass
        # 松开杆
        self.motion.openHand("LHand")
        time.sleep(2)
        self.motion.closeHand("LHand")
        self.motion.rest()

    def killTask(self, taskName):
        retTextList = os.popen("ps aux | grep %s"%taskName)
        try:
            pidList = []
            for line  in retTextList:
                pidList.append(line)
            if len(pidList) > 0:
                cmdList = pidList[0].split()
                pidNum = cmdList[1]
                print pidNum
                os.system("kill -9 %s"%pidNum)
        except:
            print "kill error"

def main(ip, port):
    '''
    Main entry point
    :param ip:
    :param port:
    :return:
    '''
    # We need this broker to be able to construct
    # NAOqi module and subscribe to other modules
    # The broker must stay alive until the program exists
    myBroker = ALBroker("myBrokerForStop",
                        "0.0.0.0", # listen to anyone
                        0,         # find a free port and use it
                        ip,        # parent broker IP
                        port)#parent broker port

    global ReactToTouch
    try:
        ReactToTouch = ReactToTouch("ReactToTouch")
        # input()
        time.sleep(55 * 60)
        myBroker.shutdown()
    except Exception,e:
        myBroker.shutdown()
        exit(0)
    finally:
        myBroker.shutdown()
if __name__ == "__main__":
   try:
       main(IP, 9559)
   except Exception,e:
       try:
           main(IP, 9559)
       except Exception, e:
           exit(0)
