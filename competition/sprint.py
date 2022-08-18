import multiprocessing
import sys
import os
import time
import cv2
from multiprocessing import Process, Pipe

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
from class_Motion import Motion1 as Motion

class Competition:
    pass

class Sprint(Competition):
    def __init__(self):
        self.path_to_camera_config = "/home/pi/kondo_fira22/Camera_calibration/mtx.yaml"
        self.glob = Glob(SIMULATION, current_work_directory)
        self.motion = Motion(self.glob)
        self.motion.activation()
        self.motion.falling_Flag = 0
        self.number_of_cycles = 3000000 #30
        self.motion.simThreadCycleInMs = 20 # 20
        self.motion.amplitude = 32 #32
        self.motion.fr1 = 4 # 4
        self.motion.fr2 = 10 # 10
        ##self.motion.initPoses = self.motion.fr2 
        self.motion.gaitHeight = 220 # 190
        self.motion.stepHeight = 40  # 20
        self.stepLength = 70 #70 
        self.forward = True
        self.stopDistance = 0.2
        self.stepIncrement = 10
        self.stepDecrement = 10
        self.maxStepLengthBack = -70
        
        self.parent_conn, child_conn = Pipe()
        self.cam_proc = Process(target=self.process_vision, args=(child_conn,))
        
        self.rvec = [[[0,0,0]]]
        self.tvec = [[[0,0,0]]]
        self.stopFlag = False

    def aruco_init(self):
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
    
    def aruco_position(self, img):
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, self.arucoDict,
                        parameters=self.arucoParams)
        # print(corners)
        # print(self.sensor.camera_matrix, self.sensor.dist_matrix)
        if corners != []:
            rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(corners[0], 0.17, self.sensor.camera_matrix, self.sensor.dist_matrix)
        else : return [[[0,0,0]]],[[[0,0,0]]]    
        return rvec, tvec

    def process_vision(self, conn):
        self.sensor = KondoCameraSensor(self.path_to_camera_config)
        self.aruco_init()
        while not self.stopFlag:
            img = self.sensor.snapshot().img
            rvec, tvec = self.aruco_position(img)
            conn.send((rvec, tvec))

    def run_forward_1(self):
        stepLength1 = 0
        self.motion.walk_Initial_Pose()
        self.cam_proc.start()
        time.sleep(2)
        for cycle in range(self.number_of_cycles):
            
            self.rvec, self.tvec = self.parent_conn.recv()
            #if cycle ==0 : stepLength1 = self.stepLength/4
            #if cycle ==1 : stepLength1 = self.stepLength/2
            #if cycle ==2 : stepLength1 = self.stepLength/4 * 3
              
            if cycle<10 and stepLength1 < self.stepLength:

                stepLength1 += self.stepIncrement
           
            if 0 < self.tvec[0][0][2] < self.stopDistance and stepLength1 > self.maxStepLengthBack:
                stepLength1 = -self.stepDecrement
            step_rot = 0 if -0.3 < self.rvec[0][0][1] < 0.1 else -max(-0.6,min(0.4,self.rvec[0][0][1]))
            self.motion.refresh_Orientation()
            print(f"step_l {stepLength1} step_rot {step_rot}")
            # print("rvec : ", self.rvec)
            # print("tvec : ", self.tvec)
            self.motion.walk_Cycle(stepLength1,
                                    0,
                                    0,
                                    cycle, 
                                    self.number_of_cycles)
        self.motion.walk_Final_Pose()
        self.stopFlag = True
        self.cam_proc.join()

    def __del__(self):
        self.stopFlag = True
        self.cam_proc.join()
if __name__ == "__main__":
    sprint = Sprint()
    sprint.run_forward_1()    

