

import sys
import os
import math
import json
import time
import cv2

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
if 1:
    # will be running on openMV
    import pyb
    current_work_directory += '/'
    SIMULATION = 2                                         # 0 - Simulation without physics, 1 - Simulation synchronous with physics, 2 - live on open
sys.path.append( current_work_directory + 'Soccer/')
sys.path.append( current_work_directory + 'Soccer/Motion/')
SIMULATION=2
from class_Motion import Glob
from reload import KondoCameraSensor

if SIMULATION == 2:
    from class_Motion import Motion1 as Motion

class Competition:
    pass

class Sprint(Competition):
    def __init__(self):   
        self.glob = Glob(SIMULATION, current_work_directory)
        self.motion = Motion(self.glob)
        self.motion.activation()
        self.motion.falling_Flag = 0
        self.number_Of_Cycles = 30 #30
        self.motion.simThreadCycleInMs = 20 # 20
        self.motion.amplitude = 32 #32
        self.motion.fr1 = 4 # 4
        self.motion.fr2 = 10 # 10
        ##self.motion.initPoses = self.motion.fr2 
        self.motion.gaitHeight = 190 # 190
        self.motion.stepHeight = 40  # 20
        self.stepLength = 70 #70 
        self.sensor = KondoCameraSensor(path_to_config)
        self.aruco_init()
    def aruco_init(self):
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
    def aruco_position(self, img):
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, self.arucoDict,
                        parameters=self.arucoParams)
        tvec, rvec = cv2.aruco.estimatePoseSingleMarkers(corners[0], 0.05, mtx, dist)
        print("rvec : ", rvec)
        print("tvec : ", tvec)    
        return tvec, rvec
    def run_forward(self):
        while (True):
            img = self.sensor.snapshot().img
            rvec, tvec = self.aruco_position(img)
            rotation = 0

            self.motion.walk_Cycle(self.stepLength,0,rotation,3, 10)
if __name__ == "__main__":
    sprint = Sprint()
    sprint.run_forward()    

