#  Walking engine for Starkit Kondo OpenMV
#  Copyright STARKIT Soccer team of MIPT

import sys, os
import math, time, json

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
if True:
    current_work_directory += '/'
    with open("simulator_lib_directory.txt", "r") as f:
        simulator_lib_directory = f.read()
    simulator_lib_directory = simulator_lib_directory.replace('\\', '/')
    if simulator_lib_directory[-1] == "\n":
        simulator_lib_directory = simulator_lib_directory[:-1] 
    sys.path.append(simulator_lib_directory)
    import random
    try:
        import sim, threading
    except:
        print(sys.path)
else:
    import starkit
    sys.path.append('/')

sys.path.append( current_work_directory + 'Soccer/')
sys.path.append( current_work_directory + 'Soccer/Motion/')


from class_Motion import *
from class_Motion import Motion1
from compute_Alpha_v3 import Alpha

class Motion_sim(Motion1):
    def __init__(self, glob):
        self.yaw_timer = time.perf_counter()
        self.FRAMELENGTH = 0.02
        import reload as re
        self.re = re
        import cv2 as cv2
        self.cv2 = cv2
        import random as random
        self.random = random
        import sim as vr
        self.sim = vr
        import numpy as np
        self.np = np
        import matplotlib.pyplot as plt
        self.plt = plt
        import keyboard as keyboard
        self.keyboard = keyboard
        self.pause_key_is_pressed = False
        self.stop_key_is_pressed = False
        self.keyboard.on_press_key('p', self.on_pause)
        self.keyboard.on_press_key('s', self.on_stop)
        self.Dummy_HData =[]
        self.Dummy_1_YawData = []
        self.Dummy_1_PitchData = []
        self.Dummy_1_RollData = []
        self.BallData =[]
        self.timeElapsed = 0
        self.trims = []
        self.jointHandle = []
        self.Dummy_HHandle = 0
        self.Dummy_1Handle = 0
        self.BallHandle = 0
        self.Ballposition = []
        self.position_o =[]
        self.position_l = []
        self.position_r = []
        self.clientID = -1
        self.sim_step_counter = 0
        self.Vision_Sensor_Display_On = True
        super().__init__(glob)
        
    def on_pause(self, e):
        self.pause_key_is_pressed = True

    def on_stop(self, e):
        self.stop_key_is_pressed = True

    def wait_sim_step(self):
        while True:
            self.sim.simxGetIntegerParameter(self.clientID, self.sim.sim_intparam_program_version, self.sim.simx_opmode_buffer)
            tim = self.sim.simxGetLastCmdTime(self.clientID)
            #print ('Simulation time: ', tim)
            if tim > self.sim_step_counter:
                self.sim_step_counter = tim 
                break
            time.sleep(0.001)
            if self.stop_key_is_pressed:
                self.sim_Stop()
                time.sleep(0.1)
                self.sim_Disable()
                sys.exit(0)

    def simulateMotion(self, number = 0, name = ''):
        #mot = [(0,'Initial_Pose'),(1,0),(2,0),(3,0),(4,0),(5,'Get_Up_Left'),
        #   (6,'Soccer_Get_UP_Stomach_N'),(7,0),(8,'Soccer_Walk_FF'),(9,0),(10,0),
        #   (11,0),(12,0),(13,0),(14,'Soccer_Small_Jump_Forward'),(15,0),
        #   (16,0),(17,0),(18,'Soccer_Kick_Forward_Right_Leg'),(19,'Soccer_Kick_Forward_Left_Leg'),(20,0),
        #   (21,'Get_Up_From_Defence'),(22,0),(23,'PanaltyDefenceReady_Fast'), (24,'PenaltyDefenceF'),(25,0),
        #   (26,0),(27,0),(28,0),(29.0),(30,'Soccer_Walk_FF0'),
        #   (31,'Soccer_Walk_FF1'), (32,'Soccer_Walk_FF2'), (33,'Soccer_Get_UP_Stomach'), (34,'Soccer_Get_UP_Face_Up'),
        #   (35,'Get_Up_Right'), (36,'PenaltyDefenceR'), (37,'PenaltyDefenceL')]
        # start the simulation
        if number > 0 and name == '': name = self.MOTION_SLOT_DICT[number]
        with open(current_work_directory + "Soccer/Motion/motion_slots/" + name + ".json", "r") as f:
            slots = json.loads(f.read())
        mot_list = slots[name]
        i=0
        for motion in mot_list:
            if  self.falling_Flag ==3: return
            activePoseOld = []
            l1 = len(self.activePose)
            for ind in range(len(self.activePose)): activePoseOld.append(self.activePose[ind])
            if len(activePoseOld) < 25:
                activePoseOld.append(0)
                activePoseOld.append(0)
            self.activePose =[]
            for j in range(len(motion) - 1):
                    self.activePose.append(0.017*motion[j+1]*0.03375)
            pulseNum = int(motion[0]*self.FRAMELENGTH * 1000 / self.simThreadCycleInMs)
            for k in range (pulseNum):
                self.sim.simxPauseCommunication(self.clientID, True)
                for j in range(len(motion) - 1):
                    tempActivePose = activePoseOld[j]+(self.activePose[j]-activePoseOld[j])*k/pulseNum
                    returnCode = self.sim.simxSetJointTargetPosition(self.clientID, self.jointHandle[j] ,
                                 tempActivePose*self.FACTOR[j] , self.sim.simx_opmode_streaming)
                self.sim.simxPauseCommunication(self.clientID, False)
                self.sim.simxSynchronousTrigger(self.clientID)
        return


    def sim_Start(self):
        #print ('Simulation started')
        # self.sim.simxFinish(-1) # just in case, close all opened connections
        simThreadCycleInMs = 5
        if self.glob.SIMULATION == 3 or self.glob.SIMULATION  == 0:
            self.clientID=self.sim.simxStart('127.0.0.1', -19997, True, True, 5000, simThreadCycleInMs) # Connect to V-REP
        if self.glob.SIMULATION == 1:
            self.clientID=self.sim.simxStart('127.0.0.1', -20000, True, True, 5000, simThreadCycleInMs) # Connect to V-REP
        if self.clientID!=-1:
            print ('Connected to remote API server')
        else:
            print ('Failed connecting to remote API server')
            print ('Program ended')
            exit(0)
            ## Collect Joint Handles and trims from model
        returnCode, self.Dummy_HHandle = self.sim.simxGetObjectHandle(self.clientID, 'Dummy_H', self.sim.simx_opmode_blocking)
        returnCode, self.Dummy_1Handle = self.sim.simxGetObjectHandle(self.clientID, 'Dummy1' , self.sim.simx_opmode_blocking)
        returnCode, self.BallHandle = self.sim.simxGetObjectHandle(self.clientID, 'Ball', self.sim.simx_opmode_blocking)
        returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_streaming)
        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_streaming)
        returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_streaming)
        returnCode, Dummy_1position= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_streaming)
        returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_streaming)
        returnCode, self.VisionHandle = self.sim.simxGetObjectHandle(self.clientID, 'Vision_sensor', self.sim.simx_opmode_blocking)
        returnCode, resolution, image_Data = self.sim.simxGetVisionSensorImage(self.clientID, self.VisionHandle, 0 ,self.sim.simx_opmode_streaming)
        for i in range(len(self.ACTIVEJOINTS)):
            returnCode, handle= self.sim.simxGetObjectHandle(self.clientID, self.ACTIVEJOINTS[i], self.sim.simx_opmode_blocking)
            self.jointHandle.append(handle)
            returnCode, position= self.sim.simxGetJointPosition(self.clientID, handle, self.sim.simx_opmode_blocking)
            self.trims.append(position)
            self.activePose.append(position)
        if self.glob.SIMULATION == 1:
            self.sim.simxSynchronous(self.clientID,True)
        #if self.glob.SIMULATION == 3:
        self.sim.simxGetIntegerParameter(self.clientID, self.sim.sim_intparam_program_version, self.sim.simx_opmode_streaming)

    def sim_Progress(self,simTime):  # simTime in seconds
        if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
            for i in range(int(simTime*1000//self.simThreadCycleInMs)):
                returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                self.Dummy_HData.append(Dummy_Hposition)
                self.refresh_Orientation()
                self.Dummy_1_YawData.append(self.imu_body_yaw())
                self.Dummy_1_PitchData.append(self.body_euler_angle['pitch'])
                self.Dummy_1_RollData.append(self.body_euler_angle['roll'])
                returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
                self.BallData.append(self.Ballposition)
                returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                #print(quaternion_to_euler_angle(Dummy_Hquaternion))
                self.timeElapsed = self.timeElapsed +1
                if self.glob.SIMULATION == 1 : self.sim.simxSynchronousTrigger(self.clientID)
                if self.glob.SIMULATION == 3 : 
                    time.sleep(0.005)
                    self.wait_sim_step() 

    def sim_Stop(self):
        if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
            self.sim.simxStopSimulation(self.clientID,self.sim.simx_opmode_oneshot)
                    # return to initial pose
            for j in range(len(self.ACTIVEJOINTS)):
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                   returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                self.jointHandle[j] , self.trims[j], self.sim.simx_opmode_oneshot)
                else: returnCode = self.sim.simxSetJointPosition(self.clientID,
                                   self.jointHandle[j] , self.trims[j], self.sim.simx_opmode_oneshot)
    def print_Diagnostics(self):
        Dummy_HDataX =[]
        Dummy_HDataY =[]
        Dummy_HDataZ =[]
        for i in range (self.timeElapsed):
            Dummy_HDataX.append( self.Dummy_HData[i][0])
            Dummy_HDataY.append(self.Dummy_HData[i][1])
            Dummy_HDataZ.append(self.Dummy_HData[i][2])
        BallDataX =[]
        BallDataY =[]
        BallDataZ =[]
        for i in range (self.timeElapsed):
            BallDataX.append( self.BallData[i][0])
            BallDataY.append(self.BallData[i][1])
            BallDataZ.append(self.BallData[i][2])
        print('exitFlag' ,self.exitFlag)
        rng = self.np.arange(self.timeElapsed)
        #rng = self.np.arange(len(self.position_o))
        len_YawData = len(self.Dummy_1_YawData)
        fig,ax = self.plt.subplots(figsize=(10,6))
        pos_l = self.np.ma.masked_where(self.position_l == 0, self.position_l )
        self.plt.plot(rng,Dummy_HDataX, label = 'Body X')
        #self.plt.plot(rng,Dummy_HDataY, label = 'Body Y')
        #self.plt.plot(Dummy_HDataX,Dummy_HDataY, label = 'Body Y')
        #self.plt.plot(rng,Dummy_HDataZ, label = 'Body Z')
        #self.plt.plot(rng,BallDataX, label = 'Ball X')
        #self.plt.plot(rng,BallDataY, label = 'Ball Y')
        #self.plt.plot(rng,BallDataZ, label = 'Ball Z')
        #self.plt.plot(rng,self.Dummy_1_YawData, label = 'Body Yaw')
        #self.plt.plot(rng,self.Dummy_1_PitchData, label = 'Body Pitch')
        #self.plt.plot(rng,self.Dummy_1_RollData, label = 'Body Roll')
        #self.plt.plot(rng,self.position_o, label = 'Center Of Mass')
        #self.plt.errorbar(rng, pos_l, yerr = 7.5, label = 'Left Step')
        #self.plt.errorbar(rng,self.position_r, yerr = 7.5, label = 'Right Step')
        ax.legend(loc='upper left')
        ax.grid(True)
        #ax.set_ylim(-85, 85)
        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
            self.plt.show()
            #break
        print('exitFlag' ,self.exitFlag)

    def sim_Disable(self):            # Now close the connection to Sim:
        time.sleep(0.2)
        self.sim.simxFinish(self.clientID)

    def vision_Sensor_Get_Image(self):
        returnCode, resolution, image_Data = self.sim.simxGetVisionSensorImage(self.clientID, self.VisionHandle, 0 ,self.sim.simx_opmode_buffer)
        nuimg = self.np.array(image_Data, dtype=self.np.uint8)
        nuimg.shape = (resolution[1],resolution[0],3)
        nuimg1 = self.cv2.cvtColor(nuimg, self.cv2.COLOR_RGB2BGR)
        img = self.np.flip(nuimg1, 1)
        return img

    def vision_Sensor_Display(self, img):
        if self.Vision_Sensor_Display_On:
            self.cv2.imshow('Vision Sensor', img)
            #self.cv2.waitKey(0) & 0xFF
            res = self.cv2.waitKey(0)
            if res == 115:
                print('you have pressed "s"')
                token = str(int(self.random.random()*10000))
                filename = current_work_directory + "Soccer/" + token + '.png'
                isWritten = self.cv2.imwrite(filename, img)

if __name__=="__main__":
    print('This is not main module!')


