

import sys
import os
import math
import json
import time
import cv2
from competition import Competition

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
if 1:
    # will be running on openMV
    import pyb
    current_work_directory += '/'
    SIMULATION = 2                                         # 0 - Simulation without physics, 1 - Simulation synchronous with physics, 2 - live on open
sys.path.append( current_work_directory + 'Soccer/')
sys.path.append( current_work_directory + 'Motion/')
SIMULATION=2
from class_Motion import Glob
from reload import KondoCameraSensor

if SIMULATION == 2:
    from class_Motion import Motion1 as Motion

class Sprint(Competition):
    def __init__(self, button, path_to_camera_config):   
        super().__init__(button)
        self.motion.activation()
        self.motion.falling_Flag = 0
        self.number_of_cycles = 3000000 #30
        self.motion.simThreadCycleInMs = 20 # 20
        self.motion.amplitude = 32 #32
        self.motion.fr1 = 4 # 4
        self.motion.fr2 = 10 # 10
        ##self.motion.initPoses = self.motion.fr2 
        self.motion.gaitHeight = 190 # 190
        self.motion.stepHeight = 40  # 20
        self.stepLength = 70 #70 
        self.sensor = KondoCameraSensor(path_to_camera_config)
        self.aruco_init()
    def aruco_init(self):
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
    def aruco_position(self, img):
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, self.arucoDict,
                        parameters=self.arucoParams)
        print(corners)
        print(self.sensor.camera_matrix, self.sensor.dist_matrix)
        if corners != []:
            tvec, rvec = cv2.aruco.estimatePoseSingleMarkers(corners[0], 0.05, self.sensor.camera_matrix, self.sensor.dist_matrix)
        else : return [[[0,0,0]]],[[[0,0,0]]]
        print("rvec : ", rvec)
        print("tvec : ", tvec)    
        return tvec, rvec

    def run_forward_1(self):
        for cycle in range(self.number_of_cycles):
            img = self.sensor.snapshot().img
            rvec, tvec = self.aruco_position(img)
            rotation = 0 
            if cycle ==0 : stepLength1 = self.stepLength/4
            if cycle ==1 : stepLength1 = self.stepLength/2
            if cycle ==2 : stepLength1 = self.stepLength/4 * 3
            self.motion.walk_Cycle(stepLength1,0,max(rvec[0][0][1],0.1),cycle, self.number_of_cycles)

if __name__ == "__main__":
    default_button = 1
    sprint = Sprint(default_button, "/home/pi/kondo_fira22/Camera_calibration/mtx.yaml")
    sprint.run_forward_1()    

