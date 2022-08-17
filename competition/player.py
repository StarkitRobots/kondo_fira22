

import sys
import os
import math
import json
import time


current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
if 1:
    # will be running on openMV
    import pyb
    current_work_directory += '/'
    SIMULATION = 2                                         # 0 - Simulation without physics, 1 - Simulation synchronous with physics, 2 - live on openMV
else:
    # will be running on desktop computer
    current_work_directory += '/'
    import threading
    SIMULATION = 1                                          # 3 - Simulation streaming with physics, 4 - simulation webots

sys.path.append( current_work_directory + 'Soccer/')
sys.path.append( current_work_directory + 'Soccer/Motion/')
SIMULATION=2
from class_Motion import Glob

if SIMULATION == 2:
    from class_Motion import Motion1 as Motion
else:
    from class_Motion import *
    from class_Motion_sim import*
    from class_Motion_sim import Motion_sim as Motion



class Player():
    def __init__(self, role):
        self.role = role   
        self.glob = Glob(SIMULATION, current_work_directory)
        self.motion = None
        self.dl_params ={}

    def simulation(self):
        self.motion = Motion(self.glob)
        self.motion.sim_Start()
        #if self.role == 'run_test':
        #    self.motion.neck_tilt = -2000
        #    if self.glob.SIMULATION == 2:
        #        self.motion.kondo.setUserParameter(20,self.motion.neck_tilt)
        #        #pyb.delay(400)
        #    else:
        #        returnCode = self.motion.sim.simxSetJointTargetPosition(self.motion.clientID,
        #                    self.motion.jointHandle[22] , self.motion.neck_tilt * self.motion.TIK2RAD * self.motion.FACTOR[22],
        #                   self.motion.sim.simx_opmode_oneshot)  # Шея Наклон
        #        for j in range(20):
        #            self.motion.sim.simxSynchronousTrigger(self.motion.clientID)
        self.common_init()
        #eval('self.' + self.role + '_main_cycle()')
        if self.role == 'run_test': self.run_test_main_cycle(1)
        if self.role == 'kondo_walk': self.kondo_walk_main_cycle(1)
        if self.role == 'weight_lifting': self.weight_lifting_main_cycle(1)
        if self.role == 'run_turf_test': self.run_turf_test_main_cycle(1)
        if self.role == 'sidestep_test': self.sidestep_test_main_cycle()
        if self.role == 'balancing_test': self.balancing_test_main_cycle()
        if self.role == 'basketball': self.basketball_main_cycle()
        if SIMULATION != 0:
            self.motion.sim_Progress(1)
        self.motion.sim_Stop()
        if SIMULATION != 0:
            self.motion.print_Diagnostics()
        self.motion.sim_Disable()

    def real(self, button):
        self.motion = Motion(self.glob)
        #if self.role == 'run_test':
        #    self.motion.neck_tilt = -2000
        #    if self.glob.SIMULATION == 2:
        #        self.motion.kondo.setUserParameter(20,self.motion.neck_tilt)
        #        #pyb.delay(400)
        #    else:
        #        returnCode = self.motion.sim.simxSetJointTargetPosition(self.motion.clientID,
        #                    self.motion.jointHandle[22] , self.motion.neck_tilt * self.motion.TIK2RAD * self.motion.FACTOR[22],
        #                   self.motion.sim.simx_opmode_oneshot)  # Шея Наклон
        #        for j in range(20):
        #            self.motion.sim.simxSynchronousTrigger(self.motion.clientID)
        # pressed_button = self.motion.push_Button(button)
        pressed_button = button
        self.common_init()
        if self.role == 'run_test': self.run_test_main_cycle(pressed_button)
        if self.role == 'kondo_walk': self.kondo_walk_main_cycle(pressed_button)
        if self.role == 'weight_lifting': self.weight_lifting_main_cycle(pressed_button)
        if self.role == 'run_turf_test': self.run_turf_test_main_cycle(pressed_button)
        if self.role == 'sidestep_test': self.sidestep_test_main_cycle()
        if self.role == 'balancing_test': self.balancing_test_main_cycle()
        if self.role == 'triple_jump': self.triple_jump_main_cycle()
        if self.role == 'basketball': self.basketball_main_cycle()

    def common_init(self):
        self.motion.activation()
        self.motion.falling_Flag = 0

    def basketball_main_cycle(self):
        #self.motion.play_Soft_Motion_Slot(name = 'Ball_tennis_throw_v7_2')
        self.motion.kondo.motionPlay(78)

    def triple_jump_main_cycle(self):
        self.motion.kondo.motionPlay(77)

    def run_test_main_cycle(self, pressed_button):
        if pressed_button == 1:             # fast step
            number_Of_Cycles = 30 #30
            self.motion.simThreadCycleInMs = 20 # 20
            self.motion.amplitude = 32 #32
            self.motion.fr1 = 4 # 4
            self.motion.fr2 = 10 # 10
            ##self.motion.initPoses = self.motion.fr2 
            self.motion.gaitHeight = 190 # 190
            self.motion.stepHeight = 20  # 20
            stepLength = 80 #88
        if pressed_button == 2  :   
            number_Of_Cycles = 10 #30
            self.motion.simThreadCycleInMs = 20
            self.motion.amplitude = 32 #32
            self.motion.fr1 = 8 # 4
            self.motion.fr2 = 12
            ##self.motion.initPoses = self.motion.fr2 
            self.motion.gaitHeight = 170 #200
            self.motion.stepHeight = 3 #40  # 20
            stepLength = 100 #150 #100
        sideLength = 0
        #self.motion.first_Leg_Is_Right_Leg = False
        if self.motion.first_Leg_Is_Right_Leg: invert = -1.5
        else: invert = 1
        while True:
            print (self.motion.glob.SIMULATION)
            self.motion.walk_Initial_Pose()
            number_Of_Cycles += 1
            for cycle in range(number_Of_Cycles):
                #if cycle > 1: self.glob.camera_ON = True
                if not self.motion.falling_Flag == 0: break
                stepLength1 = stepLength
                if cycle ==0 : stepLength1 = stepLength/4
                if cycle ==1 : stepLength1 = stepLength/2
                if cycle ==2 : stepLength1 = stepLength/4 * 3
                self.motion.refresh_Orientation()
                rotation = 0 + invert * self.motion.body_euler_angle['yaw'] * 1.0
                #if rotation < 0: rotation *= 5
                rotation = self.motion.normalize_rotation(rotation)
                #rotation = -0.5
                self.motion.walk_Cycle(stepLength1,0,0,cycle, number_Of_Cycles)
                '''if self.motion.i_see_ball:
                    stepLength1 = stepLength/3 * 2
                    self.motion.refresh_Orientation()
                    rotation = 0 + invert * self.motion.body_euler_angle['yaw'] * 1.0
                    rotation = self.motion.normalize_rotation(rotation)
                    self.motion.walk_Cycle(stepLength1,sideLength, rotation, 1, 3)
                    stepLength1 = stepLength/3
                    self.motion.refresh_Orientation()
                    rotation = 0 + invert * self.motion.body_euler_angle['yaw'] * 1.0
                    rotation = self.motion.normalize_rotation(rotation)
                    self.motion.walk_Cycle(stepLength1,sideLength, rotation, 1, 3)
                    stepLength1 = -stepLength/3
                    self.motion.refresh_Orientation()
                    rotation = 0 + invert * self.motion.body_euler_angle['yaw'] * 1.0
                    rotation = self.motion.normalize_rotation(rotation)
                    self.motion.walk_Cycle(stepLength1,sideLength, rotation, 1, 3)
                    stepLength1 = -stepLength/3 * 2
                    self.motion.refresh_Orientation()
                    rotation = 0 + invert * self.motion.body_euler_angle['yaw'] * 1.0
                    rotation = self.motion.normalize_rotation(rotation)
                    self.motion.walk_Cycle(stepLength1,sideLength, rotation, 1, 3)
                    for back_cycle in range(5):
                        stepLength1 = -stepLength
                        self.motion.refresh_Orientation()
                        rotation = 0 + invert * self.motion.body_euler_angle['yaw'] * 1.0
                        rotation = self.motion.normalize_rotation(rotation)
                        self.motion.walk_Cycle(stepLength1,sideLength, rotation, back_cycle, 5)
                    break'''
            if not self.motion.falling_Flag == 0:
                if self.motion.falling_Flag == 3: 
                    print('STOP!')
                    return
                else: 
                    print('FALLING!!!', self.motion.falling_Flag)
                    self.motion.falling_Flag = 0
                    continue
            self.motion.walk_Final_Pose()
            return

    def kondo_walk_main_cycle(self, pressed_button):
        step_number = 5
        self.motion.play_Soft_Motion_Slot(name = 'Walk_B_Initial')
        for i in range(step_number):
            self.motion.play_Soft_Motion_Slot(name = 'Walk_B_Cycle1')
        return

    def weight_lifting_main_cycle(self, pressed_button):
        def walk_straight(number_Of_Cycles = 0, stepLength = 0):
            self.motion.walk_Initial_Pose()
            number_Of_Cycles += 2
            for cycle in range(number_Of_Cycles):
                stepLength1 = stepLength
                if cycle ==0 or cycle == number_Of_Cycles-1 : stepLength1 = stepLength/3
                if cycle ==1 or cycle == number_Of_Cycles-2 : stepLength1 = stepLength/3 * 2
                self.motion.refresh_Orientation()
                rotation = - self.motion.body_euler_angle['yaw'] * 1.2
                rotation = self.motion.normalize_rotation(rotation)
                self.motion.walk_Cycle(stepLength1, 0, rotation,cycle, number_Of_Cycles)
            self.motion.walk_Final_Pose()

        def look_at_weights():
            self.motion.move_head(0, -2665)
            self.motion.vision_Sensor_Display(self.motion.vision_Sensor_Get_Image())
            self.motion.move_head(0, 0)

        self.motion.amplitude = 20
        self.motion.stepHeight = 30
        self.motion.params['BODY_TILT_AT_WALK'] = 0.01

        walk_straight(number_Of_Cycles = 37, stepLength = 4)          #41 # Стартовый проход
        look_at_weights()
        self.motion.play_Soft_Motion_Slot(name = 'Weight_Lift_1')
        if SIMULATION == 2 : pyb.delay(1000)
        else:  self.motion.sim_Progress(1)
        self.motion.keep_hands_up = True
        self.motion.ztr0 = - self.motion.gaitHeight
        self.motion.ztl0 = - self.motion.gaitHeight
        self.motion.params['BODY_TILT_AT_WALK'] = -0.1
        walk_straight(number_Of_Cycles = 18, stepLength = 30)

        self.motion.play_Soft_Motion_Slot(name = 'Weight_Lift_2')
        if SIMULATION == 2 : pyb.delay(1000)
        else:  self.motion.sim_Progress(1)
        self.motion.keep_hands_up = True
        self.motion.ztr0 = - self.motion.gaitHeight
        self.motion.ztl0 = - self.motion.gaitHeight
        self.motion.params['BODY_TILT_AT_WALK'] = 0.0075
        walk_straight(number_Of_Cycles = 25, stepLength = 30)
        return

    def run_turf_test_main_cycle(self, pressed_button):
        if pressed_button == 2 or pressed_button ==3 :
            self.sidestep_test_main_cycle(pressed_button)
            return
        number_Of_Cycles = 40
        self.motion.amplitude = 32 #20 #48
        #self.motion.fr1 = 4 #14
        #self.motion.fr2 = 24 #32 #24
        ##self.motion.initPoses = self.motion.fr2 
        self.motion.gaitHeight = 190
        self.motion.stepHeight = 40
        stepLength = 80 #64
        sideLength = 0
        #self.motion.first_Leg_Is_Right_Leg = False
        if self.motion.first_Leg_Is_Right_Leg: invert = -1
        else: invert = 1
        while True:
            self.motion.walk_Initial_Pose()
            number_Of_Cycles += 1
            for cycle in range(number_Of_Cycles):
                if not self.motion.falling_Flag == 0: break
                stepLength1 = stepLength
                if cycle ==0 : stepLength1 = stepLength/3
                if cycle ==1 : stepLength1 = stepLength/3 * 2
                self.motion.refresh_Orientation()
                rotation = 0 + invert * self.motion.body_euler_angle['yaw'] * 1.5
                #if rotation < 0: rotation *= 5
                rotation = self.motion.normalize_rotation(rotation)
                #rotation = 0
                self.motion.walk_Cycle(stepLength1,sideLength, rotation,cycle, number_Of_Cycles)
            if not self.motion.falling_Flag == 0:
                if self.motion.falling_Flag == 3: 
                    print('STOP!')
                    return
                else: 
                    print('FALLING!!!', self.motion.falling_Flag)
                    self.motion.falling_Flag = 0
                    continue
            self.motion.walk_Final_Pose()
            return
