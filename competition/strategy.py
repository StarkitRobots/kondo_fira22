import sys
import os
import math
import json
import time
import numpy as np

#from basketball.basketball_node import Basketball

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
if 1:
    # will be running on openMV
    import pyb
    SIMULATION = 2
    current_work_directory += '/'
# 0 - Simulation without physics, 1 - Simulation synchronous with physics, 2 - live on openMV
else:
    # will be running on desktop computer
    current_work_directory += '/'
    import threading
    SIMULATION = 1                                          # 3 - Simulation streaming with physics, 4 - simulation webots

sys.path.append( current_work_directory + 'Soccer/')
sys.path.append( current_work_directory + 'Soccer/Motion/')
sys.path.append(current_work_directory + 'Soccer/Motion/Jump/')
from class_Motion import Glob
from approach import approach
from center_point import center_point
from localisation import localisation

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
        if self.role == 'run_test':
            self.motion.neck_tilt = -2000
            if self.glob.SIMULATION == 2:
                self.motion.kondo.setUserParameter(20,self.motion.neck_tilt)
                #pyb.delay(400)
            else:
                returnCode = self.motion.sim.simxSetJointTargetPosition(self.motion.clientID,
                            self.motion.jointHandle[22] , self.motion.neck_tilt * self.motion.TIK2RAD * self.motion.FACTOR[22],
                           self.motion.sim.simx_opmode_oneshot)  # Шея Наклон
                for j in range(20):
                    self.motion.sim.simxSynchronousTrigger(self.motion.clientID)
        self.common_init()
        #eval('self.' + self.role + '_main_cycle()')
        if self.role == 'run_test': self.run_test_main_cycle(2)
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
        
        if self.role == 'run_test':
            self.motion.neck_tilt = -2000
            if self.glob.SIMULATION == 2:
                self.motion.kondo.setUserParameter(20,self.motion.neck_tilt)
                #pyb.delay(400)
            else:
                returnCode = self.motion.sim.simxSetJointTargetPosition(self.motion.clientID,
                            self.motion.jointHandle[22] , self.motion.neck_tilt * self.motion.TIK2RAD * self.motion.FACTOR[22],
                           self.motion.sim.simx_opmode_oneshot)  # Шея Наклон
                for j in range(20):
                    self.motion.sim.simxSynchronousTrigger(self.motion.clientID)
        pressed_button = 1# self.motion.push_Button(button)
        
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
        
        def move_head_real(tilt, pan):

            sd_t = self.motion.kondo.ServoData()
            sd_p = self.motion.kondo.ServoData()

            sd_t.id = 12
            sd_t.sio = 2
            sd_t.data = tilt

            sd_p.id = 0
            sd_p.sio = 1
            sd_p.data = pan

            self.motion.kondo.setServoPos([sd_t, sd_p], 5)

        def rad_to_kondo(angle):
            angle = angle + (3*np.pi/4)
            turn = 3500 + int(angle*(8000/(3*np.pi/2)))
            return turn

        def get_pixels(name):
            center = []
            self.glob.camera_ON = True
            center = self.motion.check_camera(name)
            pixels = center[0]
            return pixels

        def get_distance(pixels, name):
            coords = self.motion.self_coords_from_pixels(pixels[0], pixels[1], name)
            print(f"coordinates of object: {coords}")
            return coords

        def radians_search(frequency):
            res = list(zip(np.zeros(frequency), np.linspace(-np.pi/4, np.pi/4, frequency))) + list(zip(np.zeros(frequency) - np.pi/4, np.linspace(np.pi/4, -np.pi/4, frequency)))
            return res + [[0, 0]]

        def finding(name):
            flag = False
            mediana_of_coords = (None, None)
            distance = 0
            print("Start finding ball")
            radians = radians_search(5)
            l = []
            fl = 0
            while len(l) < 2:
                for elem in radians:
                    # move_head_real(rad_to_kondo(elem[0]), rad_to_kondo(elem[1])) # NEED TO MOVE HEAD!!!
                    time.sleep(1)
                    pixels = get_pixels(name)
                    print(f"PIXELS BLYAT':{pixels}")
                    if not 600 < pixels[0] < 1000 and fl < 1 or not 400 < pixels[1] < 800 and fl < 1:
                        angle_p = pixels[0] * (np.pi / 1600)
                        angle_p = angle_p - (np.pi / 2)
                        
                        angle_t = pixels[1] * (np.pi / 1600)
                        angle_t = angle_t - (np.pi / 2)
                        
                        move_head_real(rad_to_kondo(-np.pi/8), rad_to_kondo(angle_p))
                        pixels = get_pixels(name)
                        fl += 1
                    if pixels != (None, None):
                        coords = get_distance(pixels, name)
                        print(coords)
                        l.append(coords)
                # choose the mediana of all balls in coordinates of robor and look at dispersy, check massive of all finded balls
                    else:
                        print("GOVNO")
            mediana_of_coords = tuple(np.median(np.array(l), axis = 0))
            print(f"MEDIANA: {mediana_of_coords}")
            move_head_real(rad_to_kondo(0), rad_to_kondo(0))
            if mediana_of_coords != (None, None):
                flag = True
                distance = np.sqrt(mediana_of_coords[0] ** 2 + mediana_of_coords[1] ** 2)
            return flag, mediana_of_coords, distance

        '''
        def rotate(to_rotate_deg):    #XYETAAAAAAA

            imu_start = self.imu_client().x
            imu_end = imu_start - to_rotate_deg
            imu_end %= 360
            rotation = 0.2
            print(to_rotate_deg)

            while True:
                imu = self.imu_client()
                print("imu: ", imu.x)
                print("imu end: ", imu_end)
                if np.abs(imu.x - imu_end) < 3:
                    break
                
                if imu.x - imu_end > 0:
                    if math.fabs(imu.x - imu_end) <= 180:
                        self.walk_client(True, 0, 0, rotation)
                    else:
                        self.walk_client(True, 0, 0, -rotation)
                if  imu.x - imu_end <= 0:
                    print("Negative")
                    if math.fabs(imu.x - imu_end) <= 180:
                        self.walk_client(True, 0, 0, -rotation)
                    else:
                        self.walk_client(True, 0, 0, rotation)
                
            self.walk_client(False, 0, 0, 0)
        '''

        def turn_to(angle):
            print("Start turning to ball/basketcase")
            number_of_cycles = 10
            for cycle in range(number_of_cycles):                
                self.motion.walk_Cycle(0,
                                        0,
                                        0.25 + angle / number_of_cycles,
                                        cycle, 
                                        number_of_cycles)
            print(str(angle))
            # rotate(degree)      #THIS FUNCTION NEEDS ANOTHER VIEW
            time.sleep(1)
        def test_forward(number_of_cycles):
            print("Start going. Number of cycles : {number_of_cycles} ")
            stepLength = 30
            self.motion.frames_per_cycle = 100
            self.motion.walk_Initial_Pose()
            time.sleep(3)
            self.motion.frames_per_cycle = self.default_frames_per_cycle
            for cycle in range(number_of_cycles):                
                self.motion.walk_Cycle(stepLength,
                                        0,
                                        0.25,
                                        cycle, 
                                        number_of_cycles)
        def test_rotation(number_of_cycles):
            print(f"Start turning to ball/basketcase. Number of cycles :{number_of_cycles}")
            
            for cycle in range(number_of_cycles):                
                self.motion.walk_Cycle(0,
                                        0,
                                        0.25,
                                        cycle, 
                                        number_of_cycles)

        def test_series(n):
            data_cycles = []
            data_distance = []
            for i in range(1,
            n,2):
                test_forward(i)
                data_distance.append(float(input()))
                data_cycles.append(i)
            print(data_distance)
            return data_cycles, data_distance 
        def draw_graphics(data_cycles,data_distance):
            import numpy as np
            import matplotlib.pyplot as plt
            plt.plot(data_cycles,data_distance)
            plt.show()
        def go_to(coords, distance):
            print("Start going")
            stepLength = 70
            number_of_cycles = int(distance*4200/stepLength)
            self.motion.walk_Initial_Pose()
            time.sleep(3)
            for cycle in range(number_of_cycles):                
                self.motion.walk_Cycle(stepLength,
                                        0,
                                        0.25,
                                        cycle, 
                                        number_of_cycles)
            self.motion.walk_Final_Pose()
            print("end of going")
            '''
            r_x, r_y, r_theta = 1, 2, 0 #robot's coordinates
        #img = Image(cv2.imread("Soccer\\5837.png", cv2.COLOR_BGR2LAB))
        #blob = img.find_blobs([[100, 110, 140, 210, 220, 230], [50, 70, 80, 230, 240, 250], [10, 20, 30, 251, 252, 253]], 30, 30)
            
            point = [100, 100, 100]
            acc = 0.001
            cycle = 2
            number_of_cycles = 3000000 #30
            self.motion.simThreadCycleInMs = 20 # 20
            self.motion.amplitude = 32 #32
            self.motion.fr1 = 4 # 4
            self.motion.fr2 = 10 # 10
            ##self.motion.initPoses = self.motion.fr2 
            self.motion.gaitHeight = 220 # 190
            self.motion.stepHeight = 40  # 20
            stepLength = 70
            sideLength = 0
            #self.motion.first_Leg_Is_Right_Leg = False
            self.motion.walk_Initial_Pose()
            while math.fabs(r_x - point[0]) >= acc or math.fabs(r_y - point[1]) >= acc or math.fabs(r_theta - point[2]) >= acc:
                point = center_point()
                r_x = coords[0]
                r_y = coords[1]
                
                steps = approach(r_x, r_y, r_theta, point[0], point[1], point[2])
                try:
                    r_x, r_y, r_theta = localisation(steps[1])
                    print(r_x, r_y, r_theta)
                    if self.motion.first_Leg_Is_Right_Leg: invert = -1
                    else: invert = 1
                    
                    if not self.motion.falling_Flag == 0: break
                    stepLength1 = stepLength
                    if cycle == 0 : stepLength1 = stepLength/3
                    if cycle == 1 : stepLength1 = stepLength/3 * 2
                    if (r_x - point[0])**2 + (r_x - point[1])**2 <= stepLength: stepLength1 = math.sqrt((r_x - point[0])**2 + (r_x - point[1])**2)
                    #rotation = steps[0][2]
                    #rotation = self.motion.normalize_rotation(rotation) 
                    rotation = 0
                    number_of_cycles += 1
                    self.motion.walk_Cycle(stepLength1, sideLength, rotation, cycle, number_of_cycles)
                    if not self.motion.falling_Flag == 0:
                        if self.motion.falling_Flag == 3: 
                            print('STOP!')
                            return
                        else: 
                            print('FALLING!!!', self.motion.falling_Flag)
                            self.motion.falling_Flag = 0
                            continue
                    cycle += 1
                    number_of_cycles += 1
                    
                except IndexError:
                    r_x, r_y, r_theta = point[0], point[1], point[2]
            self.motion.walk_Final_Pose()
            #self.motion.simulateMotion(name = 'Kondo3_TripleJump')
                ## NEED TO GO TO THE BALL!!!
            '''
        def thinking_take(ball_coordinates, basketcase_coordinates):
            print("Start thinking and taking ball")
            basketcase_linia = ((basketcase_coordinates[0]) / (basketcase_coordinates[1]))
            angle_thinking = np.pi/2 - np.arctanh(basketcase_linia)
            if ball_coordinates != (None, None):
                distance = (math.cos(angle_thinking)) * np.sqrt(ball_coordinates[0] ** 2 + ball_coordinates[1] ** 2)
            if math.fabs(distance) > 0.16:
                # TAKE THE BALL
                self.motion.play_Soft_Motion_Slot(name = 'basketball_taking_ball')
                time.sleep(1)
                # pox, it won't bring down the holder
                return True
            elif distance < 0:
                # TAKE THE BALL
                self.motion.play_Soft_Motion_Slot(name = 'basketball_taking_ball')
                time.sleep(1)
                # 3 steps right NEED TO DO!!!
                return True
            elif distance >= 0:
                # TAKE THE BALL
                self.motion.play_Soft_Motion_Slot(name = 'basketball_taking_ball')
                time.sleep(1)
                # 3 steps left NEED TO DO!!!
                return True
            else:
                return False 

        #self.motion.play_Soft_Motion_Slot(name = 'Ball_tennis_throw_v7_2')
        #self.motion.kondo.motionPlay(78) 
        #self.motion.activation()
        #self.motion.falling_Flag = 0
        '''
        #PROVERKA OF VISION

        while True:
            pixels = get_pixels('basket')
            if pixels != (None, None):
                coords = get_distance(pixels, 'basket')
                print(f"coordinates of basket: {coords}")
            pixels = get_pixels('ball')
            if pixels != (None, None):
                coords = get_distance(pixels, 'ball')
                print(f"coordinates of ball: {coords}")
            time.sleep(10)
        
        #END OF PROVERKA OF VISION
        
        '''

        # THE BEGINING
        self.glob.camera_ON = False
        flag_ball = False
        flag_basket = False
        flag_evade = False

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

        # draw_graphics(test_series(4))

        '''
        coords = (0.2, 0)
        distance = np.sqrt(coords[0]**2 + coords[1]**2)
        print(f"DISTANCY IS {distance}")
        go_to(coords, distance)
        '''
        # self.motion.move_head(1000, -2000)
        # Finding ball and putting it to self.ball_coordinates for future approach.
        # move_head_real(rad_to_kondo(-np.pi/8), rad_to_kondo(0))
        
        flag_ball = False
        print("zdorova")
        print("ya kryg")
        while not flag_ball:
            flag_ball, ball_coords, ball_distance = finding('ball')      # (True/False), (x,y), distance
        print(f"result of finding ball: {ball_coords} and {ball_distance}")

        ball_coord_x = ball_coords[0] * 1.4
        ball_coord_y = ball_coords[1] * 1.2
        
        ball_coords = (ball_coord_x, ball_coord_y)
        print(f"POGONNYE LENGTHS ARE {ball_coords}")
        #Approaching to the ball on 0.8 of distance
        
        flag_ball = False
        angle = np.arctan(ball_coords[1] / ball_coords[0])
        turn_to(angle)    #DOESN'T WORK NOW
        # maybe do finding one more time
        go_to(ball_coords, ball_distance)  #DOESN'T WORK NOW
        
        # Correct the ball position

        while not flag_ball:
            flag_ball, ball_coords, ball_distance = finding('ball')      # (True/False), (x,y), distance
        print(f"result of finding ball: {ball_coords} and {ball_distance}")

        if ball_coords[0] >= 0.1:
            
            ball_coord_x = ball_coords[0] * 1.4
            ball_coord_y = ball_coords[1] * 1.2
            
            ball_coords = (ball_coord_x, ball_coord_y)
            print(f"POGONNYE LENGTHS ARE {ball_coords}")

        # Finally approach a ball
        
            flag_ball = False
            # turn_to(ball_coords)    #DOESN'T WORK NOW
            # maybe do finding one more time
            go_to(ball_coords, ball_distance)  #DOESN'T WORK NOW

        angle = np.arctan(ball_coords[1] / ball_coords[0])
        turn_to(ball_coords)
        '''
        # Searching for ball
        while not flag_ball:
            flag_ball, ball_coords, ball_distance = finding('ball')      # (True/False), (x,y), distance
        print(f"result of finding ball: {ball_coords} and {ball_distance}")

        # Searching for basket
        while not flag_basket:          # DOESN'T WORKING, THERE SHOULD BE MASK AND THRESHOLDING
            flag_basket, basket_coords, basket_distance = finding('basket')      # (True/False), (x,y), distance
        print(f"result of finding ball: {basket_coords} and {basket_distance}")


        self.motion.play_Soft_Motion_Slot(name = 'basketball_taking_ball')
        # Take ball and do several steps from ball holder
        
        flag_basket = False
        while not flag_evade:
            flag_evade = thinking_take(ball_coords, basket_coords)
        print("Atom avoided collision with holder")
        
        
        # Searching for basket again

        while not flag_basket:          # DOESN'T WORKING, THERE SHOULD BE MASK AND THRESHOLDING
            flag_basket, basket_coords, basket_distance = finding('basket')      # (True/False), (x,y), distance
        print(f"result of finding ball: {basket_coords} and {basket_distance}")

        # Approach basket first time

        flag_basket = False
        turn_to(basket_coords)    #DOESN'T WORK NOW
        # maybe do finding one more time
        go_to(1, basket_coords, basket_distance)  #DOESN'T WORK NOW

        # Searching for basket again

        while not flag_basket:          # DOESN'T WORKING, THERE SHOULD BE MASK AND THRESHOLDING
            flag_basket, basket_coords, basket_distance = finding('basket')      # (True/False), (x,y), distance
        print(f"result of finding ball: {basket_coords} and {basket_distance}")

        # Finally approach basket

        flag_basket = False
        turn_to(basket_coords)    #DOESN'T WORK NOW
        go_to(1, basket_coords, basket_distance)  #DOESN'T WORK NOW

        # put the ball into basketcase
        '''
        self.motion.play_Soft_Motion_Slot(name = 'basketball_taking_ball_coleni')
        
        self.motion.play_Soft_Motion_Slot(name = 'basketball_throwing_ball_new')
        
        '''
        print("HE HIT THE BALL OR NO. I don't NO")
        while True:
            n = int(input())
            basketball.go_to_ball(1, n)
        '''

    def triple_jump_main_cycle(self):
        
        self.motion.play_Soft_Motion_Slot(name = 'Kondo3_TripleJump')

    def run_test_main_cycle(self, pressed_button):
        if pressed_button == 1:             # fast step
            number_Of_Cycles = 30 #30
            self.motion.simThreadCycleInMs = 10
            self.motion.amplitude = 32 #32
            self.motion.fr1 = 4 # 4
            self.motion.fr2 = 9 # 10
            ##self.motion.initPoses = self.motion.fr2 
            self.motion.gaitHeight = 170 # 190
            self.motion.stepHeight = 30  # 20
            stepLength = 50 #88
        if pressed_button == 2  :   
            number_Of_Cycles = 30 #30
            self.motion.simThreadCycleInMs = 10
            self.motion.amplitude = 0 #32
            self.motion.fr1 = 8 # 4
            self.motion.fr2 = 12
            ##self.motion.initPoses = self.motion.fr2 
            self.motion.gaitHeight = 160
            self.motion.stepHeight = 40  # 20
            stepLength = 40
        sideLength = 0
        #self.motion.first_Leg_Is_Right_Leg = False
        if self.motion.first_Leg_Is_Right_Leg: invert = -1
        else: invert = 1
        while True:
            self.motion.walk_Initial_Pose()
            number_Of_Cycles += 1
            for cycle in range(number_Of_Cycles):
                if cycle > 1: self.glob.camera_ON = True
                if not self.motion.falling_Flag == 0: break
                stepLength1 = stepLength
                if cycle ==0 : stepLength1 = stepLength/3
                if cycle ==1 : stepLength1 = stepLength/3 * 2
                self.motion.refresh_Orientation()
                rotation = 0 + invert * self.motion.body_euler_angle['yaw'] * 1.0
                #if rotation < 0: rotation *= 5
                rotation = self.motion.normalize_rotation(rotation)
                #rotation = 0
                self.motion.walk_Cycle(stepLength1,sideLength, 0,cycle, number_Of_Cycles)
                if self.motion.i_see_ball:
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
                    break
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


if __name__=="__main__":
    player = Player('basketball')  # 'run_test', 'balancing_test', 'basketball', 'weight_lifting', 'kondo_walk'
    #player.deep_learning()
    #for i in range(1):
    #    player.test_walk()
    #if SIMULATION != 0:
    #    player.motion.print_Diagnostics()
    #player.test_walk_2()
    player.real(1)






