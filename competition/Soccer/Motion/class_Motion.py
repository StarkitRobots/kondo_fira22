#  Walking engine for Starkit Kondo OpenMV
#  Copyright STARKIT Soccer team of MIPT

import sys, os
import math, time, json
from compute_Alpha_v4 import Alpha
import starkit
import cv2
import time
#os.chdir('../..')
path_to_model = os.getcwd() + "/Soccer/Model/"
from reload import *
sys.path.append(path_to_model)
from kondo3_model import RobotModel
class Glob:
    def __init__(self, simulation, current_work_directory):
        self.camera_ON = False
        self.current_work_directory = current_work_directory
        self.SIMULATION = simulation             # 0 - Simulation without physics, 1 - Simulation with physics, 2 - live on openMV
        if self.SIMULATION == 1 or self.SIMULATION == 0 or self.SIMULATION == 3 or self.SIMULATION == 4:
            with open(current_work_directory + "Soccer/Init_params/Sim/Sim_params.json", "r") as f:
                self.params = json.loads(f.read())
            with open(current_work_directory + "Soccer/Init_params/Sim/Sim_Thresholds.json", "r") as f:
                self.TH = json.loads(f.read())
        elif self.SIMULATION == 2 :
            with open(current_work_directory + "Soccer/Init_params/Real/Real_params.json", "r") as f:
                self.params = json.loads(f.read())
            with open(current_work_directory + "Soccer/Init_params/Real/Real_Thresholds.json", "r") as f:
                self.TH = json.loads(f.read())
class Imu:
    def __init__(self):
        return
    def quaternion(self):
        return [0,0,0,0]
class Motion1:
    def __init__(self, glob):
        self.test_max_angl = None
        self.tets_min_angl = None
        self.i_see_ball = False
        self.glob = glob
        self.params = self.glob.params
        self.ACTIVESERVOS = [(10,2),(9,2),(8,2),(7,2),(6,2),(5,2),(4,1),
                (3,2),(2,2),(1,2),(0,2),(10,1),(9,1),(8,1),
                (7,1),(6,1),(5,1),(4,2),(3,1),(2,1),(1,1)]
        # self.ACTIVESERVOS = [(10,2),(9,2),(8,2),(7,2),(6,2),(5,2),(4,2),
        #         (3,2),(2,2),(1,2),(0,2),(10,1),(9,1),(8,1),
        #         (7,1),(6,1),(5,1),(4,1),(3,1),(2,1),(1,1)]
        # (10,2) Прав Стопа Вбок, (9,2) Прав Стопа Вперед,(8,2) Прав Голень, (7,2) Прав Колено,
        # (6,2)  Прав Бедро,      (5,2) Прав Таз,         (1,2) Прав Ключица,(4,2) Прав Локоть, (0,2) Торс
        # (10,1) Лев Стопа Вбок,  (9,1) Лев Стопа Вперед, (8,1) Лев Голень,  (7,1) Лев Колено
        # (6,1)  Лев Бедро,       (5,1) Лев Таз,          (1,1) Лев Ключица, (4,1) Лев Локоть

        self.servo_Trims = [0 for i in range(len(self.ACTIVESERVOS))]

        #FACTOR =  [ 1,-1,-1,1,-1,-1, 1,1,1,-1,1,-1,-1, 1,1,-1,-1, 1,1,1,-1,-1, 1]  # v2.3
        #self.FACTOR =  [ 1,1,1,-1,1,1, 1,1,1,1,1,1,1, 1,-1,1,1, 1,1,1,1, 1, 1, 1, 1]  # Surrogat 1
        self.FACTOR =  [ -1,-1,1,-1,1,1, 1,1,1,1,1,-1,-1, 1,1,-1,1, 1,1,1,1, 1, 1, 1, 1]  # Kondo-3
        a5 = 40  # мм расстояние от оси симметрии до оси сервы 5
        b5 = 0  # мм расстояние от оси сервы 5 до оси сервы 6 по горизонтали
        c5 = 0     # мм расстояние от оси сервы 6 до нуля Z по вертикали
        a6 = 0    # мм расстояние от оси сервы 6 до оси сервы 7
        a7 = 80  # мм расстояние от оси сервы 7 до оси сервы 8
        a8 = 80  # мм расстояние от оси сервы 8 до оси сервы 9
        a9 = 0  # мм расстояние от оси сервы 9 до оси сервы 10
        a10= 13  # мм расстояние от оси сервы 10 до центра стопы по горизонтали
        b10= 38  # мм расстояние от оси сервы 10 до низа стопы   26.4
        c10 = 0   # мм расстояние от оси сервы 6 до оси сервы 10 по горизонтали
        self.e10 = 55 # мм половина длины стопы
        self.SIZES = [ a5, b5, c5, a6, a7, a8, a9, a10, b10, c10 ]
        self.d10 = 53 #53.4 # расстояние по Y от центра стопы до оси робота
        self.zt0 = c5 + a6 + a7 + a8 + a9 + b10 
        limAlpha5 = [-2667, 2667]
        limAlpha6 = [-2700, 1000]
        limAlpha7 = [-3000, 3000]
        limAlpha8 = [-500, 4000]
        limAlpha9 = [-2700, 3500]
        limAlpha10 =[-2700, 2700]
        LIMALPHA = [limAlpha5, limAlpha6, limAlpha7, limAlpha8, limAlpha9, limAlpha10]
        self.MOTION_SLOT_DICT = {0:['',0], 1:['',0], 2:['',0], 3:['',0], 4:['',0], 5:['Get_Up_Inentification',7000],
                    6:['Soccer_Get_UP_Stomach_N', 5000], 7:['Soccer_Get_UP_Face_Up_N', 5000], 8:['Soccer_Walk_FF',0], 9:['',0], 10:['',0],
                    11:['',0], 12:['',0], 13:['',0], 14:['Soccer_Small_Jump_Forward',0], 15:['',0],
                    16:['',0], 17:['',0], 18:['Soccer_Kick_Forward_Right_Leg',5000], 19: ['Soccer_Kick_Forward_Left_Leg',5000], 20:['',0],
                    21:['Get_Up_From_Defence',1000], 22:['',0], 23:['PanaltyDefenceReady_Fast',500], 24:['PenaltyDefenceF',300], 25:['',0],
                    26:['',0], 27:['',0], 28:['',0], 29:['',0], 30:['Soccer_Walk_FF0',0],
                    31:['Soccer_Walk_FF1',0], 32:['Soccer_Walk_FF2',0], 33: ['Soccer_Get_UP_Stomach',0], 34:['Soccer_Get_UP_Face_Up',0],
                    35: ['Get_Up_Right',0], 36: ['PenaltyDefenceR',2000], 37: ['PenaltyDefenceL',2000],   38:[ "archery_ready" , 111],
    39: ["archery_setup" , 112],
     40 :["archery_pull" , 113]}
        self.stepLengthPlaner_is_ON = False
        self.TIK2RAD = 0.00058909
        self.slowTime   = 0.0             # seconds
        self.simThreadCycleInMs = 20
        self.frame_delay = self.glob.params['FRAME_DELAY']
        self.frames_per_cycle = self.glob.params['FRAMES_PER_CYCLE']
        self.stepLength = 0.0    # -50 - +70. Best choise 64 for forward. Maximum safe value for backward step -50.
        self.sideLength = 0.0         # -20 - +20. Side step length to right (+) and to left (-)
        self.rotation = 0           # -45 - +45 degrees Centigrade per step + CW, - CCW.
        self.first_Leg_Is_Right_Leg = True
        # Following paramenetrs Not recommended for change
        self.amplitude = 32          # mm side amplitude (maximum distance between most right and most left position of Center of Mass) 53.4*2
        self.fr1 =8                  # frame number for 1-st phase of gait ( two legs on floor)
        self.fr2 = 12                # frame number for 2-nd phase of gait ( one leg in air)
        self.gaitHeight = 160         # Distance between Center of mass and floor in walk pose
        self.stepHeight = 32.0       # elevation of sole over floor
        self.initPoses = 400//self.simThreadCycleInMs
        self.limAlpha1 =LIMALPHA
        #self.limAlpha1[3][1]=0
        #  end of  paramenetrs Not recommended for change
        self.al = Alpha()
        self.exitFlag = 0
        self.falling_Flag = 0
        self.neck_pan = 0
        self.old_neck_pan = 0
        self.body_euler_angle ={}
        self.old_neck_tilt = 0
        self.direction_To_Attack = 0
        self.zero_pozition_body_roll = 0
        self.zero_pozition_body_pitch = 0
        self.zero_pozition_neck_tilt = 0
        self.last_pitch = 0
        self.activePose = [0 for i in range(25)]
        self.keep_hands_up = False
        self.xtr = 0
        self.ytr = -self.d10   #-53.4
        self.ztr = -self.gaitHeight
        self.xnr = 55                       # foot tip
        self.ynr = -self.d10   #-53.4       # foot tip
        self.znr = -self.gaitHeight         # foot tip
        self.xpr = -55                      # foot heel
        self.ypr = -self.d10   #-53.4       # foot heel
        self.zpr = -self.gaitHeight         # foot heel
        self.xr = 0
        self.yr = 0
        self.zr = -1
        self.wr = 0
        self.xtl = 0
        self.ytl = self.d10   # 53.4
        self.ztl = -self.gaitHeight
        self.xnl = 55                       # foot tip
        self.ynl = self.d10   # 53.4        # foot tip
        self.znl = -self.gaitHeight         # foot tip
        self.xnl = -55                      # foot heel
        self.ynl = self.d10   # 53.4        # foot heel
        self.znl = -self.gaitHeight         # foot heel
        self.xl = 0
        self.yl = 0
        self.zl = -1
        self.wl = 0
        self.ztr0 = -self.zt0 # -223.1
        self.ztl0 = -self.zt0 # -223.1
        self.dx0_last = 0
        self.euler_angle = {}
        self.modified_roll = 0
        self.constant_tilt = 0 #0.065
        self.body_tilt_param = 180  #1/250   #1/170
        self.correct_pitch_param = 0 #0.05   # 0.03
        self.roll_correction_param = 0 # 0.3
        self.y_correction_param = 0
        self.dynamic_tilt_param = 0.04
        self.robot_In_0_Pose = False
        self.tempangle = 0
        self.tempangles =[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.neck_tilt = 0
        self.neck_pan = 0
        self.anglesR = []
        self.anglesL = []
        self.right_hip_pitch_target_position = 0
        self.left_hip_pitch_target_position = 0
        self.ACTIVEJOINTS = ['Leg_right_10','Leg_right_9','Leg_right_8','Leg_right_7','Leg_right_6','Leg_right_5','hand_right_4',
            'hand_right_3','hand_right_2','hand_right_1','Tors1','Leg_left_10','Leg_left_9','Leg_left_8',
            'Leg_left_7','Leg_left_6','Leg_left_5','hand_left_4','hand_left_3','hand_left_2','hand_left_1','head0','head12', 'hand_right_11', 'hand_left_11']
        self.hand_joints =[6, 7, 8, 9, 17, 18, 19, 20, 23, 24]
        self.ACTIVEJOINTS_NO_Hands = ['Leg_right_10','Leg_right_9','Leg_right_8','Leg_right_7','Leg_right_6','Leg_right_5',
                                      'Tors1','Leg_left_10','Leg_left_9','Leg_left_8',
                                      'Leg_left_7','Leg_left_6','Leg_left_5']
        if self.glob.SIMULATION == 2 :
            #from button_test import wait_for_button_pressing
            import starkit
            import pyb
            from kondo3_controller import Kondo3Rcb4BaseLib as Rcb4BaseLib
            from pyb import UART
            # from machine import I2C
            from bno055 import BNO055, AXIS_P7
            #import sensor, image
            #self.wait_for_button_pressing = wait_for_button_pressing
            self.starkit = starkit
            # self.i2c = I2C(2)
            self.bno055 = BNO055
            self.imu = None
            self.imu = Imu()# BNO055(self.i2c, mode = 0x08)
            #self.green_led = LED(2)
            # self.pin2 = Pin('P2', Pin.IN, Pin.PULL_UP)
            #uart = UART(self.glob.params['UART_PORT'], self.glob.params['UART_SPEED'], timeout=1000, parity=0)
            #uart = pyb.UART(1, 1250000, parity=0)
            #k = kondo.Kondo()
            ##k.init(uart)
            self.kondo = Rcb4BaseLib()
            self.kondo.open('/dev/ttyAMA2', 1250000, 1.3)
            # self.clock = time.clock()
            self.clock = time.time()
            # self.kondo.motionPlay(25)
            self.pyb = pyb
            self.image = None
            self.cv2 = cv2
            print("__init__ of Motion1 is done")
               #-------------------------------------------------------------------------------------------------------------------------------
    def set_servo_pos(self,id,sio,angle):
        servoDatas = self.kondo.ServoData(id,sio,angle*1698 + 7500)
        self.kondo.setServoPos (self,[servoDatas], 10)
        pass

    def check_camera(self, name):
        if self.glob.camera_ON:
            thresholds = [(30, 100, 15, 127, 15, 127), # generic_red_thresholds
                            (30, 100, -64, -8, -32, 32), # generic_green_thresholds
                            (0, 30, 0, 64, -128, 0)] # generic_blue_thresholds
            if self.glob.SIMULATION == 2 :
                img = self.sensor.snapshot()
                self.cv2.imshow('image', img.img)
                self.cv2.waitKey(10)
                #for blob in img.find_blobs(thresholds, pixels_threshold=20, area_threshold=20, merge=True):
                #    print('blob.code:', blob.code())
            else:
                img_ = self.vision_Sensor_Get_Image()
                img = self.re.Image(img_)
                self.cv2.imshow('image', img_)
                self.cv2.waitKey(10)
            if name == 'ball':
                coords = img.find_blobs([self.glob.TH['orange ball']['th']], pixels_threshold=self.glob.TH['orange ball']['pixel'], area_threshold=self.glob.TH['orange ball']['area'], merge=True)
                print(f"number of found objects: {len(coords)}")
            if name == 'basket':
                coords = img.find_blobs([self.glob.TH['red basket']['th']], pixels_threshold=self.glob.TH['red basket']['pixel'], area_threshold=self.glob.TH['red basket']['area'], merge=True)
                print(f"number of found objects: {len(coords)}")
            if name == 'stripe':
                coords = img.find_blobs([self.glob.TH['yellow stripe']['th']], pixels_threshold=self.glob.TH['yellow stripe']['pixel'], area_threshold=self.glob.TH['yellow stripe']['area'], merge=True)
                print(f"number of found objects: {len(coords)}")
            if name == 'archery':
                center_coords = img.find_target_center([self.glob.TH['archery']['thblue']], self.glob.TH['archery']['thyellow'], area_threshold=self.glob.TH['archery']['thred'], merge=True)
                if center_coords != (None, None):       print(f"Coordinates of center of target are {center_coords}")
                return center_coords

            if name == 'ball' or name == 'basket':
                center = []
                for coord in coords:
                    center0 = (coord.cx(), coord.cy())  #pixel's (x, y) coordinates of object's center
                    center.append(center0)
                    print(f"object's center's coordinates: {center0}")
                if len(center) != 0:
                    return center
                else:
                    return [(None, None)]
            elif name == 'stripe':
                center_of_bottom = []
                for coord in coords:
                    c_bottom_x = coord.cx()
                    c_bottom_y = coord.y() + coord.h()
                    center_of_bottom0 = (c_bottom_x, c_bottom_y)   #(coord.cx(), coord.cy())  #pixel's (x, y) coordinates of object's center
                    center_of_bottom.append(center_of_bottom0)
                    print(f"object's center's coordinates: {center_of_bottom0}")
                if len(center_of_bottom) != 0:
                    return center_of_bottom
                else:
                    return [(None, None)]
    '''
        if img.find_blobs([self.glob.TH['orange ball']['th']], pixels_threshold=self.glob.TH['orange ball']['pixel'],
            area_threshold=self.glob.TH['orange ball']['area'], merge=True):
            print('I see ball')
            self.i_see_ball = True

    '''

    def self_coords_from_pixels(self, pixel_x, pixel_y, name):
        robot_model = RobotModel(self.glob)
        robot_model.update_camera_pan_tilt(self.neck_pan, self.neck_tilt)
        #robot_model.update_camera_pan_tilt(0, motion_client0)
        print((self.params["HEIGHT_OF_CAMERA"] + self.params["HEIGHT_OF_NECK"])/1000)
        if name == 'ball':
            height = 0
            #height = 0.289
        elif name == 'basket':
            height = 0.285 # MB???
        elif name == 'stripe':
            height = 0
        elif name == 'barbell':
            print("WRITE THE HEIGHT OF BARBELL")
            height = 0.15   # default height, NEED to calculate
        return robot_model.image2self(pixel_x, pixel_y, height)

    def imu_body_yaw(self):
        yaw = self.neck_pan*self.TIK2RAD + self.euler_angle['yaw']
        yaw = self.norm_yaw(yaw)
        return yaw

    def norm_yaw(self, yaw):
        yaw %= 2 * math.pi
        if yaw > math.pi:  yaw -= 2* math.pi
        if yaw < -math.pi: yaw += 2* math.pi
        return yaw

    def quaternion_to_euler_angle(self, quaternion):
        euler_angle = {}
        w,x,y,z = quaternion
        ysqr = y*y
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.atan2(t0,t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.atan2(t3,t4)
        euler_angle['yaw'] = Z
        euler_angle['pitch'] = Y
        euler_angle['roll'] = X
        return euler_angle

    def push_Button(self, button):
        pressed_button = button.wait_for_button_pressing()
        print("нажато")
        return pressed_button
        #ala = 0
        #while(ala==0):
        #    if (self.pin2.value()== 0):   # нажатие на кнопку 2 на голове
        #        ala = 1
        #        print("нажато")
        #self.pyb.delay(1000)

    def play_Motion_Slot(self, name = ''):
        if self.glob.SIMULATION == 2:
            for key in self.MOTION_SLOT_DICT:
                if self.MOTION_SLOT_DICT[key][0] == name:
                    self.kondo.motionPlay(key)
                    self.pyb.delay(self.MOTION_SLOT_DICT[key][1])
        else:
            self.simulateMotion(name = name)

    def play_Soft_Motion_Slot(self, name = ''):             # the slot from file will be played in robot 
        ACTIVESERVOS = [(10,2),(9,2),(8,2),(7,2),(6,2),(5,2),(4,2),
                (3,2),(2,2),(1,2),(0,2),(10,1),(9,1),(8,1),
                (7,1),(6,1),(5,1),(4,1),(3,1),(2,1),(1,1),(0,1),(12,2),(11,2),(11,1)]
        # (10,2) Прав Стопа Вбок, (9,2) Прав Стопа Вперед,(8,2) Прав Голень, (7,2) Прав Колено,
        # (6,2)  Прав Бедро,      (5,2) Прав Таз,         (1,2) Прав Ключица,(4,2) Прав Локоть, (0,2) Торс
        # (10,1) Лев Стопа Вбок,  (9,1) Лев Стопа Вперед, (8,1) Лев Голень,  (7,1) Лев Колено
        # (6,1)  Лев Бедро,       (5,1) Лев Таз,          (1,1) Лев Ключица, (4,1) Лев Локоть
        # (0,1)  Голова поворот,  (12,2) Голова наклон    (11,2) Прав Кисть  (11,1) Лев Кисть
        if self.glob.SIMULATION == 2:
            with open(self.glob.current_work_directory + "Soccer/Motion/motion_slots/" + name + ".json", "r") as f:
                slots = json.loads(f.read())
            motion_list = slots[name]
            for motion in motion_list:
                servoDatas = []
                for i in range(len(motion)-1):
                    pos = int(motion[i+1]) + 7500 #- self.servo_Trims[i]
                    servoDatas.append( self.kondo.ServoData(ACTIVESERVOS[i][0], ACTIVESERVOS[i][1],pos))
                #servoDatas = self.reOrderServoData(servoDatas)
                frames_number = int(motion[0]) 
                a=self.kondo.setServoPos (servoDatas, frames_number)
                self.pyb.delay(25 * frames_number)
                #self.pyb.delay(250 )
        else:
            self.simulateMotion(name = name)

    def falling_Test(self):
        if self.glob.SIMULATION == 0 or self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3 or self.glob.SIMULATION == 4:
            if self.pause_key_is_pressed:
                #self.lock.acquire()
                if self.glob.SIMULATION == 3:
                    self.sim.simxPauseSimulation(self.clientID, self.sim.simx_opmode_oneshot)
                self.pause_key_is_pressed = False
                while (True):
                    if self.pause_key_is_pressed:
                        #self.lock.release()
                        if self.glob.SIMULATION == 3:
                            self.sim.simxStartSimulation(self.clientID, self.sim.simx_opmode_oneshot)
                        self.pause_key_is_pressed = False
                        break
            if self.stop_key_is_pressed:
                print('Simulation STOP by keyboard')
                if self.glob.SIMULATION == 0 or self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                    self.sim_Stop()
                    time.sleep(0.1)
                    self.sim_Disable()
                sys.exit(0)
            self.refresh_Orientation()
            if (self.body_euler_angle['pitch']) > 1:
                self.falling_Flag = 1                   # on stomach
                self.simulateMotion(name = 'Soccer_Get_UP_Stomach_N')
            if (self.body_euler_angle['pitch']) < -1:
                self.falling_Flag = -1                  # face up
                self.simulateMotion(name = 'Soccer_Get_UP_Face_Up')
            if 1 < (self.body_euler_angle['roll']) < 2:
                self.falling_Flag = -2                  # on right side
                self.simulateMotion(name = 'Get_Up_Right')
            if -2 < (self.body_euler_angle['roll']) < -1:
                self.falling_Flag = 2                   # on left side
                self.simulateMotion(name = 'Get_Up_Left')
        if self.glob.SIMULATION == 2:
            #ad = self.kondo.getAllAdData()
            #print(ad)
            #if ad[0] == True:
            #ad3 = ad[1][3]
            #ad4 = ad[1][4]
            return
            ad3 = self.kondo.getAdData(3)
            ad4 = self.kondo.getAdData(4)
            if ad3 < 200:
                self.falling_Flag = 1     # on stomach
                #self.kondo.motionPlay(6)
                self.play_Soft_Motion_Slot(name = 'Soccer_Get_UP_Stomach_N')
                self.activePose = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                #self.pyb.delay(6000)
            if ad3 > 450:
                self.falling_Flag = -1    # face up
                #self.kondo.motionPlay(7)
                self.play_Soft_Motion_Slot(name = 'Soccer_Get_UP_Face_Up')
                self.activePose = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                #self.pyb.delay(6000)
            if ad4 > 400:
                self.falling_Flag = -2    # on right side
                #self.kondo.motionPlay(5)
                self.play_Soft_Motion_Slot(name = 'Get_Up_Right')
                self.activePose = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                #self.pyb.delay(8000)
            if ad4 < 160:
                self.falling_Flag = 2     # on left side
                #self.kondo.motionPlay(5)
                self.play_Soft_Motion_Slot(name = 'Get_Up_Left')
                self.activePose = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
                #self.pyb.delay(8000)

        return self.falling_Flag

    def computeAlphaForWalk(self,sizes, limAlpha, hands_on = True ):
        angles =[]
        anglesR=[]
        anglesL=[]
        if self.glob.SIMULATION == 2:
            anglesR = self.starkit.alpha_calculation(-self.xtr,self.ytr,self.ztr,self.xr,self.yr,self.zr,self.wr, sizes, limAlpha)
            anglesL = self.starkit.alpha_calculation(-self.xtl,-self.ytl,self.ztl,self.xl,-self.yl,self.zl,-self.wl, sizes, limAlpha)
            #anglesR = self.al.compute_Alpha_v4(-self.xtr,self.ytr,self.ztr,self.xr,self.yr,self.zr,self.wr, sizes, limAlpha)
            #anglesL = self.al.compute_Alpha_v4(-self.xtl,-self.ytl,self.ztl,self.xl,-self.yl,self.zl,-self.wl, sizes, limAlpha)
        else:
            anglesR = starkit.alpha_calculation(-self.xtr,self.ytr,self.ztr,self.xr,self.yr,self.zr,self.wr, sizes, limAlpha)
            anglesL = starkit.alpha_calculation(-self.xtl,-self.ytl,self.ztl,self.xl,-self.yl,self.zl,-self.wl, sizes, limAlpha)
            #anglesR = list(self.al.compute_Alpha_v4(-self.xtr,self.ytr,self.ztr,self.xr,self.yr,self.zr,self.wr, sizes, limAlpha))
            #anglesL = list(self.al.compute_Alpha_v4(-self.xtl,-self.ytl,self.ztl,self.xl,-self.yl,self.zl,-self.wl, sizes, limAlpha))
        if anglesR != []: 
            self.anglesR = anglesR
        else:
            print('No IK solution')
        if anglesL != []: 
            self.anglesL = anglesL
        else:
            print('No IK solution')
        if len(self.anglesR)>1:
            for i in range(len(self.anglesR)):
                if len(self.anglesR)==1: break
                if self.anglesR[0][2] > 0 and self.anglesR[1][2] < 0: 
                    self.anglesR.pop(0)
                else: self.anglesR.pop(1)
        #elif len(self.anglesR)==0:
        #    return[]
        if len(self.anglesL)>1:
            for i in range(len(self.anglesL)):
                if len(self.anglesL)==1: break
                if self.anglesL[0][2] > 0 and self.anglesL[1][2] < 0: self.anglesL.pop(0)
                else: self.anglesL.pop(1)
        #elif len(self.anglesL)==0:
        #    return[]
        if self.first_Leg_Is_Right_Leg == True:
            for j in range(6):
                if j == 3: angles.append(-self.anglesR[0][j])
                else:      angles.append(self.anglesR[0][j])
            if hands_on: angles.append(1.745)               #angles.append(0.873 + self.xtl/90 )  #
            else: angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(-0.2)               #
            else: angles.append(0.0)
            if hands_on: angles.append(0.524 - self.xtl/57.3)       #angles.append(0.300 - self.xtl/57.3)   #
            else: angles.append(0.0)
            angles.append(0.0)
            #for j in range(5): angles.append(0.0)
            for j in range(6): 
                if j == 1 or j == 2 or j == 5: angles.append(self.anglesL[0][j])
                else: angles.append(-self.anglesL[0][j])
            #for j in range(4): angles.append(0.0)
            if hands_on: angles.append(-1.745)          #angles.append(-0.873 + self.xtl/90)   #
            else: angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(0.2)               #
            else: angles.append(0.0)
            if hands_on: angles.append(-0.524 + self.xtr/57.3)      #angles.append(-0.300 + self.xtr/57.3)   #
            else: angles.append(0.0)
        else:
            for j in range(6):
                if j == 1 or j == 2 or j == 5: angles.append(-self.anglesL[0][j])
                else: angles.append(self.anglesL[0][j])
            if hands_on: angles.append(1.745)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(0.524 - self.xtr/57.3)
            else: angles.append(0.0)
            angles.append(0.0)                                  # Tors
            for j in range(6): 
                if j== 3: angles.append(self.anglesR[0][j])
                else:     angles.append(-self.anglesR[0][j])
            if hands_on: angles.append(-1.745)
            else: angles.append(0.0)
            angles.append(0.0)
            angles.append(0.0)
            if hands_on: angles.append(-0.524 + self.xtl/57.3)
            else: angles.append(0.0)
        for i in range(len(angles)):
            self.activePose[i] = angles[i]
        return angles

    def activation(self):
        if self.glob.SIMULATION == 2:
            #yaw, roll, pitch = self.imu.euler()
            #if 0<= yaw < 180: yaw = -yaw
            #if 180<= yaw <=360: yaw = 360 -yaw
            #self.euler_angle['yaw'] = math.radians(yaw)
            #self.euler_angle['pitch'] = math.radians(pitch)
            #self.euler_angle['roll'] = math.radians(roll)

            head_quaternuon_raw = self.imu.quaternion()
            head_quaternion = self.quat_multi(head_quaternuon_raw, [0.707, 0, 0.707, 0])
            self.euler_angle = self.quaternion_to_euler_angle(head_quaternion)
            self.euler_angle['yaw'] += math.pi/2
            self.euler_angle['roll'] += math.pi/2
            self.head_quaternion_2_body_euler_angle(head_quaternion)
            self.body_euler_angle['yaw'] += math.pi/2
            self.body_euler_angle['roll'] += math.pi/2
        else:
            time.sleep(0.1)
            if self.glob.SIMULATION != 0:
                self.sim.simxStartSimulation(self.clientID,self.sim.simx_opmode_oneshot)
            returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
            head_quaternion = self.from_vrep_quat_to_conventional_quat(Dummy_Hquaternion)
            self.euler_angle = self.quaternion_to_euler_angle(head_quaternion)
            self.head_quaternion_2_body_euler_angle(head_quaternion)
            returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
            self.Dummy_HData.append(Dummy_Hposition)
            returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
        self.direction_To_Attack += self.body_euler_angle['yaw']
        self.direction_To_Attack = self.norm_yaw(self.direction_To_Attack)
        self.zero_pozition_body_roll = self.body_euler_angle['roll']
        self.zero_pozition_body_pitch = self.body_euler_angle['pitch']
        self.zero_pozition_neck_tilt = self.neck_tilt

    def reOrderServoData(self, servoDatas):
        if self.keep_hands_up:
            order = [0, 7, 1, 8, 2, 9, 3, 10, 4, 11, 5, 12, 6]
        else:
            order = [0, 11, 1, 12, 2, 13, 3, 14, 4, 15, 5, 16, 6, 17, 7, 18, 8, 19, 9, 20, 10]
        servoDatasOrdered = []
        for orderNumber in order:
            servoDatasOrdered.append(servoDatas[orderNumber])
        return servoDatasOrdered

    def walk_Initial_Pose(self):
        self.dx0_last = 0
        self.robot_In_0_Pose = False
        if not 0 == 0:
            if self.falling_Flag == 3: print('STOP!')
            else: print('FALLING!!!', self.falling_Flag)
            return[]
        xr_old, xl_old, yr_old, yl_old = self.xr, self.xl, self.yr, self.yl
        # correction of body tilt forward
        self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK'] 
        self.xtr = self.xtl = 0
        amplitude = self.amplitude #70
        framestep = self.simThreadCycleInMs//10
        for j in range (self.initPoses):
            if self.glob.SIMULATION == 2: start1 = self.pyb.millis()
            self.ztr = self.ztr0 - j*(self.ztr0+self.gaitHeight)/self.initPoses
            self.ztl = self.ztl0 - j*(self.ztl0+self.gaitHeight)/self.initPoses
            self.ytr = -self.d10 - j*amplitude/4 /self.initPoses
            self.ytl =  self.d10 - j*amplitude/4 /self.initPoses
            angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1 )
            self.right_hip_pitch_target_position = angles[3]
            self.left_hip_pitch_target_position = angles[14]
            #if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
            else:
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.keep_hands_up:
                            if i in self.hand_joints : continue
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                            returnCode = self.sim.simxSetJointPosition(self.clientID,
                                         self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                         self.sim.simx_opmode_oneshot)
                    self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID,
                                              self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                        #self.Dummy_HData.append(Dummy_Hposition)
                        #self.timeElapsed = self.timeElapsed +1
                        # print(Dummy_Hposition)
                    if self.glob.SIMULATION == 1:
                        self.sim.simxSynchronousTrigger(self.clientID)
                elif self.glob.SIMULATION == 2:
                    servoDatas = []
                    for i in range(len(angles)):
                        if self.keep_hands_up:
                            if i in self.hand_joints : continue
                        pos = int(angles[i]*1698 + 7500)
                        servoDatas.append( self.kondo.ServoData(self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos))
                    print(servoDatas)
                    servoDatas = self.reOrderServoData(servoDatas)
                    #start2 = self.pyb.millis()
                    if j == 0:
                        a=self.kondo.setServoPos (servoDatas, 10)
                        self.pyb.delay(250)
                    else:
                        a=self.kondo.setServoPos (servoDatas, self.frames_per_cycle)
                    #print(servoDatas)
                    #print(clock.avg())
                    time1 = self.pyb.elapsed_millis(start1)
                    #time2 = self.pyb.elapsed_millis(start2)
                    if time1 < self.frame_delay: self.pyb.delay(self.frame_delay - time1)
        self.xr, self.xl, self.yr, self.yl = xr_old, xl_old, yr_old, yl_old

    def walk_Cycle(self, stepLength,sideLength, rotation,cycle, number_Of_Cycles):
        self.robot_In_0_Pose = False
        if not 0 == 0:
            #self.local.quality =0
            if self.falling_Flag == 3: print('STOP!')
            else: print('FALLING!!!', self.falling_Flag)
            return[]
        self.stepLength = stepLength #+ self.motion_shift_correction_x
        self.sideLength = sideLength #- self.motion_shift_correction_y
        self.rotation = math.degrees(rotation)
        #tmp1 = self.first_Leg_Is_Right_Leg
        #if rotation>0 or sideLength<0:  self.first_Leg_Is_Right_Leg = False
        #else: self.first_Leg_Is_Right_Leg = True
        rotation = -self.rotation/222 * 0.23 / self.params['ROTATION_YIELD']
        alpha = 0
        alpha01 = math.pi/self.fr1*2
        frameNumberPerCycle = 2*self.fr1+2*self.fr2
        framestep = self.simThreadCycleInMs//10
        xtl0 = self.stepLength * (1 - (self.fr1 + self.fr2 + 2 * framestep) / (2*self.fr1+self.fr2+ 2 * framestep)) # * 1.5     # 1.5 - podgon
        # xtr0 = self.stepLength * (1/2 - (self.fr1 + self.fr2 + 2 * framestep ) / (2*self.fr1+self.fr2+ 2 * framestep))
        xtr0 = self.stepLength * (1 - (self.fr1 + self.fr2 + 2 * framestep ) / (2*self.fr1+self.fr2+ 2 * framestep))
        dx0_typical = self.stepLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep        # CoM propulsion forward per framestep
        dy0_typical = self.sideLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep        # CoM propulsion sideways per framestep
        xr_old, xl_old, yr_old, yl_old = self.xr, self.xl, self.yr, self.yl
        # correction of body tilt forward
        #self.xr = self.params['BODY_TILT_AT_WALK'] - self.body_euler_angle['pitch'] * 0.3 #* stepLength / 120 - self.body_euler_angle['pitch'] * 0.1
        #self.xl = self.params['BODY_TILT_AT_WALK'] - self.body_euler_angle['pitch'] * 0.3 #* stepLength / 120 - self.body_euler_angle['pitch'] * 0.1 #
        # correction of sole skew depending on side angle of body when step pushes land
        self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
        clearance = 64 / 180 * self.gaitHeight
        if self.glob.SIMULATION == 2: test1 = self.pyb.millis()
        for iii in range(0,frameNumberPerCycle,framestep):
            self.xr, self.xl = self.params['BODY_TILT_AT_WALK'] * stepLength / 120, self.params['BODY_TILT_AT_WALK'] * stepLength / 120
            if self.glob.SIMULATION == 2: 
                start1 = self.pyb.millis()
                print("timer : ", self.pyb.elapsed_millis(test1))
            if 0<= iii < self.fr1 :                                              # FASA 1
                alpha = alpha01 * (iii/2+0.5*framestep)
                #alpha = alpha01 * iii/2
                S = (self.amplitude/2 + self.sideLength/2 )*math.cos(alpha)
                self.ytr = S - self.d10 + self.sideLength/2
                self.ytl = S + self.d10 + self.sideLength/2
                self.ztl = -self.gaitHeight
                self.ztr = -self.gaitHeight
                if cycle ==0: continue
                else: dx0 = dx0_typical
                self.xtl = xtl0 - dx0 - dx0 * iii/framestep
                self.xtr = xtr0 - dx0 - dx0 * iii/framestep

            if self.fr1+self.fr2<=iii<2*self.fr1+self.fr2 :                     # FASA 3
                alpha = alpha01 * ((iii-self.fr2)/2+0.5*framestep)
                #alpha = alpha01 * (iii-self.fr2)/2
                S = (self.amplitude/2 + self.sideLength/2)*math.cos(alpha)
                self.ytr = S - self.d10 - self.sideLength/2
                self.ytl = S + self.d10 + self.sideLength/2
                self.ztl = -self.gaitHeight
                self.ztr = -self.gaitHeight
                dx0 = dx0_typical
                self.xtl -= dx0
                self.xtr -= dx0

            if self.fr1<= iii <self.fr1+self.fr2:                               # FASA 2
                self.ztr = -self.gaitHeight + self.stepHeight
                #if self.fr1 + 2*framestep <= iii <= self.fr1 + 4*framestep : self.ztr += 5
                if cycle ==0:
                    dx = self.stepLength/(self.fr2- 2 * framestep)*framestep/2
                    dx0 = dx0_typical
                    dy = self.sideLength/self.fr2*framestep
                    dy0 = dy0_typical
                else:
                    dx = self.stepLength/(self.fr2- 2 * framestep)*framestep #* 0.75
                    dx0 = dx0_typical
                    dy = self.sideLength/self.fr2*framestep
                    dy0 = dy0_typical
                if iii==self.fr1:
                    self.xtr -= dx0
                    #self.ytr = S - 64 + dy0
                    self.ytr = S - self.d10 + dy0
                elif iii == (self.fr1 +self.fr2 - framestep):
                    self.xtr -= dx0
                    self.ytr = S - self.d10 + 2*dy0 - self.sideLength
                else:
                    self.xtr += dx
                    self.ytr = S - clearance + dy0 - dy*self.fr2/(self.fr2- 2 * framestep)*((iii - self.fr1)/2)
                    self.wr = self.wl = rotation -(iii-self.fr1)* rotation/(self.fr2- 2 * framestep)*2
                self.xtl -= dx0
                self.ytl += dy0

            if 2*self.fr1+self.fr2<= iii :                                         # FASA 4
                self.ztl = -self.gaitHeight + self.stepHeight
                #if 2*self.fr1+self.fr2< iii < 2*self.fr1 + 2*self.fr2 - framestep: self.xl += math.copysign(0.15, stepLength)
                #if 2*self.fr1+self.fr2 + 2*framestep <= iii <= 2*self.fr1+self.fr2 + 4*framestep: self.ztl += 5
                if cycle == number_Of_Cycles - 1:
                    dx0 = dx0_typical * 4 / self.fr2           # 8.75/6
                    dx = (self.stepLength*(self.fr1+self.fr2)/(4*self.fr1)+2*dx0)/(self.fr2- 2 * framestep) *framestep / 1.23076941   # 1.23076941 = podgon
                    if iii== (2*self.fr1 + 2*self.fr2 - framestep):
                        self.ztl = -self.gaitHeight
                        self.ytl = S + self.d10
                else:
                    dx = self.stepLength/(self.fr2- 2 * framestep) *framestep # * 0.75
                    dx0 = dx0_typical
                    dy = self.sideLength/(self.fr2- 2 * framestep) *framestep
                    dy0 = dy0_typical
                if iii== (2*self.fr1 + self.fr2 ):
                    self.xtl -= dx0
                    #self.ytl = S + 64 + dy0
                    self.ytl = S + self.d10 + dy0
                elif iii== (2*self.fr1 + 2*self.fr2 - framestep):
                    self.xtl -= dx0
                    self.ytl = S + self.d10 + 2*dy0 - self.sideLength
                else:
                    self.xtl += dx
                    self.ytl = S + clearance + dy0 - dy * (iii -(2*self.fr1+self.fr2) )/2
                    self.wr = self.wl = (iii-(2*self.fr1+self.fr2))* rotation/(self.fr2- 2 * framestep) *2 - rotation
                self.xtr -= dx0
                self.ytr += dy0
            #self.xtr -= 10
            #self.xtl -= 10
            angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1 )
            #self.xtr += 10
            #self.xtl += 10
            #self.check_camera()
            self.refresh_Orientation()
            print('self.body_euler_angle["pitch"]', self.body_euler_angle['pitch'])

            #print('iii = ', iii, 'ytr =', self.ytr, 'ytl =', self.ytl)
            if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
                if self.glob.SIMULATION == 2:
                    time1 = self.pyb.elapsed_millis(start1)
                else:
                    print('No IK solution')
                    if self.glob.SIMULATION == 1:
                        self.sim.simxSynchronousTrigger(self.clientID)
                    if self.glob.SIMULATION == 3: self.wait_sim_step()

            else:
                #differential_pitch = self.body_euler_angle['pitch'] - self.last_pitch
                ##if (differential_pitch > 0 and self.body_euler_angle['pitch'] > 0) or (differential_pitch < 0 and self.body_euler_angle['pitch'] < 0):
                ##   factor = 3.5
                ##else: factor = 0.5 
                #angles[3] += self.body_euler_angle['pitch'] * 3 + differential_pitch * 5
                #angles[14] -= self.body_euler_angle['pitch'] * 3 + differential_pitch * 5
                #self.last_pitch = self.body_euler_angle['pitch']
                #if self.fr1 + 2 <= iii <= self.fr1 + 8 :
                #    angles[1] = 3000 * self.TIK2RAD
                #    angles[2] = -4500 * self.TIK2RAD
                #if 2*self.fr1+self.fr2 + 2 <= iii <= 2*self.fr1+self.fr2 + 8:
                #    angles[12] = -3000 * self.TIK2RAD
                #    angles[13] = 4500 * self.TIK2RAD
                #if iii == self.fr1 + self.fr2: angles[1] = 2000 * self.TIK2RAD
                ##if iii == 2 * self.fr1 + 2 * self.fr2: angles[12] = -2000 * self.TIK2RAD
                #if iii == 0: angles[12] = -2000 * self.TIK2RAD
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.keep_hands_up:
                            if i in self.hand_joints : continue
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                           returnCode = self.sim.simxSetJointPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                    self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                        self.Dummy_HData.append(Dummy_Hposition)
                        returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
                        self.BallData.append(self.Ballposition)
                        returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
                        self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
                        #uprint(self.euler_angle)
                        self.timeElapsed = self.timeElapsed +1
                        #uprint(Dummy_Hposition)
                        #if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0:
                        #    self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                        if self.glob.SIMULATION == 1:
                            self.sim.simxSynchronousTrigger(self.clientID)
                        pass
                        #disp = []
                        #for i in range(len(angles)):
                        #    pos = int(angles[i]*1698 + 7500)
                        #    #disp.append((self.ACTIVEJOINTS[i],self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos -7500))
                        #    if 0 <= i <= 4 or 11 <= i <= 15:
                        #        disp.append(pos -7500)
                        #print(iii, self.xtr, self.ytr, self.ztr, self.xr, self.yr, self.zr, self.wr, self.xtl, self.ytl, self.ztl, self.xl, self.yl, self.zl, self.wl)
                        #print(iii, self.wr)
                elif self.glob.SIMULATION == 2:
                    servoSpeedDatas =[]
                    #if iii == 2*self.fr1+2*self.fr2 - framestep or iii == self.fr1 + self.fr2 - framestep:
                    #    servoSpeedDatas.append(self.kondo.ServoData(7, 1, 20))
                    #    servoSpeedDatas.append(self.kondo.ServoData(8, 1, 127))
                    #    servoSpeedDatas.append(self.kondo.ServoData(9, 1, 20))
                    #    servoSpeedDatas.append(self.kondo.ServoData(7, 2, 20))
                    #    servoSpeedDatas.append(self.kondo.ServoData(8, 2, 127))
                    #    servoSpeedDatas.append(self.kondo.ServoData(9, 2, 20))
                    #else:
                    for i in range(7,10):
                        servoSpeedDatas.append(self.kondo.ServoData(i, 1, 127))
                        servoSpeedDatas.append(self.kondo.ServoData(i, 2, 127))
                    self.kondo.setServoSpeed(servoSpeedDatas)
                    servoDatas = []
                    disp = []
                    for i in range(len(angles)):
#                        print(angles)
                        if self.keep_hands_up:
                            if i in self.hand_joints : continue
                        pos = int(angles[i]*1698 + 7500)
                        disp.append((self.ACTIVEJOINTS[i],self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos -7500))
                        servoDatas.append( self.kondo.ServoData(self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos))
                    servoDatas = self.reOrderServoData(servoDatas)
                    start2 = self.pyb.millis()
                    a=self.kondo.setServoPosAsync (servoDatas, self.frames_per_cycle)
                    #print('disp[4] = ', disp[4], 'disp[15]=', disp[15])
                    time1 = self.pyb.elapsed_millis(start1)
                    time2 = self.pyb.elapsed_millis(start2)
                    print('calc time =',time1 - time2, 'transfer time =', time2 )
                    self.pyb.delay(self.frame_delay - time1)
                #self.refresh_Orientation()
        # returning xr, xl, yr, yl to initial value
        self.xr, self.xl, self.yr, self.yl = xr_old, xl_old, yr_old, yl_old
        #self.local.coord_shift[0] = self.cycle_step_yield*stepLength/64/1000
        #if self.first_Leg_Is_Right_Leg:
        #    self.local.coord_shift[1] = -self.side_step_right_yield * abs(sideLength)/20/1000
        #else: self.local.coord_shift[1] = self.side_step_left_yield * abs(sideLength)/20/1000
        #self.local.coordinate_record(odometry = True, shift = True)
        #self.first_Leg_Is_Right_Leg = tmp1

    def walk_Cycle2(self, stepLength,sideLength, rotation,cycle, number_Of_Cycles):
        self.robot_In_0_Pose = False

        self.stepLength = stepLength #+ self.motion_shift_correction_x
        self.sideLength = sideLength #- self.motion_shift_correction_y
        self.rotation = math.degrees(rotation)
        #tmp1 = self.first_Leg_Is_Right_Leg
        #if rotation>0 or sideLength<0:  self.first_Leg_Is_Right_Leg = False
        #else: self.first_Leg_Is_Right_Leg = True
        rotation = -self.rotation/222 * 0.23 / self.params['ROTATION_YIELD']
        alpha = 0
        alpha01 = math.pi/self.fr1*2
        frameNumberPerCycle = 2*self.fr1+2*self.fr2
        framestep = self.simThreadCycleInMs//10
        xtl0 = self.stepLength * (1 - (self.fr1 + self.fr2 + 2 * framestep) / (2*self.fr1+self.fr2+ 2 * framestep)) * 1.5     # 1.5 - podgon
        xtr0 = self.stepLength * (1/2 - (self.fr1 + self.fr2 + 2 * framestep ) / (2*self.fr1+self.fr2+ 2 * framestep))
        dx0_typical = self.stepLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep        # CoM propulsion forward per framestep
        dy0_typical = self.sideLength/(2*self.fr1+self.fr2+ 2 * framestep)*framestep        # CoM propulsion sideways per framestep
        xr_old, xl_old, yr_old, yl_old = self.xr, self.xl, self.yr, self.yl
        # correction of body tilt forward
        #self.xr = self.params['BODY_TILT_AT_WALK'] - self.body_euler_angle['pitch'] * 0.3 #* stepLength / 120 - self.body_euler_angle['pitch'] * 0.1
        #self.xl = self.params['BODY_TILT_AT_WALK'] - self.body_euler_angle['pitch'] * 0.3 #* stepLength / 120 - self.body_euler_angle['pitch'] * 0.1 #
        # correction of sole skew depending on side angle of body when step pushes land
        self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
        clearance = 64 / 180 * self.gaitHeight
        if self.glob.SIMULATION == 2: test1 = self.pyb.millis()
        for iii in range(0,frameNumberPerCycle,framestep):
            #self.xr, self.xl = self.params['BODY_TILT_AT_WALK'] * stepLength / 120, self.params['BODY_TILT_AT_WALK'] * stepLength / 120
            if self.glob.SIMULATION == 2: 
                start1 = self.pyb.millis()
                print("timer : ", self.pyb.elapsed_millis(test1))
            if 0<= iii <self.fr1 :                                              # FASA 1
                support_leg = 'both=>left'
                alpha = alpha01 * (iii/2+0.5*framestep)
                #alpha = alpha01 * iii/2
                S = (self.amplitude/2 + self.sideLength/2 )*math.cos(alpha)
                self.ytr = S - self.d10 + self.sideLength/2
                self.ytl = S + self.d10 + self.sideLength/2
                self.ztl = -self.gaitHeight
                self.ztr = -self.gaitHeight
                if cycle ==0: continue
                else: dx0 = dx0_typical
                self.xtl = xtl0 - dx0 - dx0 * iii/framestep
                self.xtr = xtr0 - dx0 - dx0 * iii/framestep

            if self.fr1+self.fr2<=iii<2*self.fr1+self.fr2 :                     # FASA 3
                support_leg = 'both=>right'
                alpha = alpha01 * ((iii-self.fr2)/2+0.5*framestep)
                #alpha = alpha01 * (iii-self.fr2)/2
                S = (self.amplitude/2 + self.sideLength/2)*math.cos(alpha)
                self.ytr = S - self.d10 - self.sideLength/2
                self.ytl = S + self.d10 + self.sideLength/2
                self.ztl = -self.gaitHeight
                self.ztr = -self.gaitHeight
                dx0 = dx0_typical
                self.xtl -= dx0
                self.xtr -= dx0

            if self.fr1<= iii <self.fr1+self.fr2:                               # FASA 2
                support_leg = 'left'
                self.ztr = -self.gaitHeight + self.stepHeight
                if self.fr1 + 2*framestep <= iii  : self.xr += math.copysign(self.params['SWING_STEP_PITCH'], stepLength)
                #if self.fr1 + 2*framestep <= iii <= self.fr1 + 4*framestep : self.ztr += 5
                if cycle ==0:
                    dx = self.stepLength/(self.fr2- 2 * framestep)*framestep/2
                    dx0 = dx0_typical
                    dy = self.sideLength/self.fr2*framestep
                    dy0 = dy0_typical
                else:
                    dx = self.stepLength/(self.fr2- 2 * framestep)*framestep #* 0.75
                    dx0 = dx0_typical
                    dy = self.sideLength/self.fr2*framestep
                    dy0 = dy0_typical
                if iii==self.fr1:
                    self.xtr -= dx0
                    #self.ytr = S - 64 + dy0
                    self.ytr = S - self.d10 + dy0
                elif iii == (self.fr1 +self.fr2 - framestep):
                    self.xtr -= dx0
                    self.ytr = S - self.d10 + 2*dy0 - self.sideLength
                else:
                    self.xtr += dx
                    self.ytr = S - clearance + dy0 - dy*self.fr2/(self.fr2- 2 * framestep)*((iii - self.fr1)/2)
                    self.wr = self.wl = rotation -(iii-self.fr1)* rotation/(self.fr2- 2 * framestep)*2
                self.xtl -= dx0
                self.ytl += dy0

            if 2*self.fr1+self.fr2<= iii :                                         # FASA 4
                support_leg = 'right'
                self.ztl = -self.gaitHeight + self.stepHeight
                #if 2*self.fr1+self.fr2< iii < 2*self.fr1 + 2*self.fr2 - framestep: self.xl += math.copysign(0.15, stepLength)
                if 2*self.fr1+self.fr2 + 2*framestep <= iii : self.xl += math.copysign(self.params['SWING_STEP_PITCH'], stepLength)
                #if 2*self.fr1+self.fr2 + 2*framestep <= iii <= 2*self.fr1+self.fr2 + 4*framestep: self.ztl += 5
                if cycle == number_Of_Cycles - 1:
                    dx0 = dx0_typical * 4 / self.fr2           # 8.75/6
                    dx = (self.stepLength*(self.fr1+self.fr2)/(4*self.fr1)+2*dx0)/(self.fr2- 2 * framestep) *framestep / 1.23076941   # 1.23076941 = podgon
                    if iii== (2*self.fr1 + 2*self.fr2 - framestep):
                        self.ztl = -self.gaitHeight
                        self.ytl = S + self.d10
                else:
                    dx = self.stepLength/(self.fr2- 2 * framestep) *framestep # * 0.75
                    dx0 = dx0_typical
                    dy = self.sideLength/(self.fr2- 2 * framestep) *framestep
                    dy0 = dy0_typical
                if iii== (2*self.fr1 + self.fr2 ):
                    self.xtl -= dx0
                    #self.ytl = S + 64 + dy0
                    self.ytl = S + self.d10 + dy0
                elif iii== (2*self.fr1 + 2*self.fr2 - framestep):
                    self.xtl -= dx0
                    self.ytl = S + self.d10 + 2*dy0 - self.sideLength
                else:
                    self.xtl += dx
                    self.ytl = S + clearance + dy0 - dy * (iii -(2*self.fr1+self.fr2) )/2
                    self.wr = self.wl = (iii-(2*self.fr1+self.fr2))* rotation/(self.fr2- 2 * framestep) *2 - rotation
                self.xtr -= dx0
                self.ytr += dy0
            #self.xtr -= 10
            #self.xtl -= 10
            angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1 )
            #self.xtr += 10
            #self.xtl += 10
            #self.check_camera()
            self.refresh_Orientation()
            print('self.body_euler_angle["pitch"]', self.body_euler_angle['pitch'])

            #print('iii = ', iii, 'ytr =', self.ytr, 'ytl =', self.ytl)
            if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
                if self.glob.SIMULATION == 2:
                    time1 = self.pyb.elapsed_millis(start1)
                    self.pyb.delay(self.frame_delay - time1)
                else:
                    print('No IK solution')
                    if self.glob.SIMULATION == 1:
                        self.sim.simxSynchronousTrigger(self.clientID)
                    if self.glob.SIMULATION == 3: self.wait_sim_step()

            else:
                if support_leg == 'right' or support_leg == 'both=>right':
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        returnCode, alpha2= self.sim.simxGetJointPosition(self.clientID, self.jointHandle[3], self.sim.simx_opmode_blocking)
                        alpha2 *= self.FACTOR[3]
                    elif self.glob.SIMULATION == 2:
                        alpha2 = self.kondo.getSinglePos(7, 1)
                    alpha1 = -alpha2 - self.right_hip_pitch_target_position
                else:
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        returnCode, alpha2= self.sim.simxGetJointPosition(self.clientID, self.jointHandle[14], self.sim.simx_opmode_blocking)
                        alpha2 *= self.FACTOR[14]
                    elif self.glob.SIMULATION == 2:
                        alpha2 = self.kondo.getSinglePos(7, 2)
                    alpha1 = alpha2 - self.left_hip_pitch_target_position
                self.right_hip_pitch_target_position = angles[3]
                self.left_hip_pitch_target_position = angles[14]
                pitch = self.body_euler_angle['pitch']
                beta = pitch - alpha1
                if math.fabs(beta) < 0.05:
                    overhead_factor = 1
                else: overhead_factor = 1
                #print('overhead_factor:', overhead_factor)
                L1 = 16
                L2 = 10
                m1 = 0.82
                m2 = 1.2
                cos1 = math.fabs((1 + overhead_factor * m1/(2*m2))*L1/L2 * math.sin(pitch - alpha1))
                if cos1 > 1: theta = 0
                else: theta = math.acos(cos1)
                if beta >= 0: 
                    control = -math.pi/2 + theta - pitch
                else:
                    control = math.pi/2 - theta - pitch
                angles[3] -= control
                angles[14] += control
                self.last_pitch = self.body_euler_angle['pitch']
                if self.fr1 + 2 <= iii <= self.fr1 + 8 :
                    angles[1] = 3000 * self.TIK2RAD
                    angles[2] = -4500 * self.TIK2RAD
                if 2*self.fr1+self.fr2 + 2 <= iii <= 2*self.fr1+self.fr2 + 8:
                    angles[12] = -3000 * self.TIK2RAD
                    angles[13] = 4500 * self.TIK2RAD
                if iii == self.fr1 + self.fr2: angles[1] = 2000 * self.TIK2RAD
                #if iii == 2 * self.fr1 + 2 * self.fr2: angles[12] = -2000 * self.TIK2RAD
                if iii == 0: angles[12] = -2000 * self.TIK2RAD
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.keep_hands_up:
                            if i in self.hand_joints : continue
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                           returnCode = self.sim.simxSetJointPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                    self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                        self.Dummy_HData.append(Dummy_Hposition)
                        returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
                        self.BallData.append(self.Ballposition)
                        returnCode, Dummy_1quaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
                        self.body_euler_angle = self.quaternion_to_euler_angle(Dummy_1quaternion)
                        #uprint(self.euler_angle)
                        self.timeElapsed = self.timeElapsed +1
                        #uprint(Dummy_Hposition)
                        #if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0:
                        #    self.vision_Sensor_Display(self.vision_Sensor_Get_Image())
                        if self.glob.SIMULATION == 1:
                            self.sim.simxSynchronousTrigger(self.clientID)
                elif self.glob.SIMULATION == 2:
                    servoSpeedDatas =[]
                    for i in range(7,10):
                        servoSpeedDatas.append(self.kondo.ServoData(i, 1, 127))
                        servoSpeedDatas.append(self.kondo.ServoData(i, 2, 127))
                    self.kondo.setServoSpeed(servoSpeedDatas)
                    servoDatas = []
                    disp = []
                    for i in range(len(angles)):
                        if self.keep_hands_up:
                            if i in self.hand_joints : continue
                        pos = int(angles[i]*1698 + 7500)
                        disp.append((self.ACTIVEJOINTS[i],self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos -7500))
                        servoDatas.append( self.kondo.ServoData(self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos))
                    servoDatas = self.reOrderServoData(servoDatas)
                    start2 = self.pyb.millis()
                    a=self.kondo.setServoPos (servoDatas, self.frames_per_cycle)
                    #print('disp[4] = ', disp[4], 'disp[15]=', disp[15])
                    time1 = self.pyb.elapsed_millis(start1)
                    time2 = self.pyb.elapsed_millis(start2)
                    print('calc time =',time1 - time2, 'transfer time =', time2 )
                    self.pyb.delay(self.frame_delay - time1)
                #self.refresh_Orientation()
        # returning xr, xl, yr, yl to initial value
        self.xr, self.xl, self.yr, self.yl = xr_old, xl_old, yr_old, yl_old
        #self.local.coord_shift[0] = self.cycle_step_yield*stepLength/64/1000
        #if self.first_Leg_Is_Right_Leg:
        #    self.local.coord_shift[1] = -self.side_step_right_yield * abs(sideLength)/20/1000
        #else: self.local.coord_shift[1] = self.side_step_left_yield * abs(sideLength)/20/1000
        #self.local.coordinate_record(odometry = True, shift = True)
        #self.first_Leg_Is_Right_Leg = tmp1

    def walk_Cycle3(self, stepLength,sideLength, rotation,cycle, number_Of_Cycles, secondStepLength = 1000):
        self.robot_In_0_Pose = False

        self.stepLength = stepLength
        self.sideLength = sideLength
        self.rotation = math.degrees(rotation)
        rotation = -self.rotation/286
        alpha01 =  math.pi/self.fr2
        frameNumberPerStep = self.fr2
        framestep = self.simThreadCycleInMs//10
        xt0 = self.stepLength /2
        xtr0 = - self.stepLength /2
        dx0_typical = self.stepLength/self.fr2 * framestep        # CoM propulsion forward per framestep
        dy0 = self.sideLength / self.fr2 * framestep        # CoM propulsion sideways per framestep
        dy = self.sideLength /self.fr2 * framestep
        xr_old, xl_old, yr_old, yl_old = self.xr, self.xl, self.yr, self.yl
        # correction of sole skew depending on side angle of body when step pushes land
        self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
        # correction of body tilt forward
        self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']   #
        wr_old = self.wr
        wl_old = self.wl
        wr_target = - rotation
        wl_target = - rotation
        #self.constant_tilt = 0 #0.065
        #self.body_tilt_param = 1/180  #1/250   #1/170
        #self.correct_pitch_param = 0 #0.05   # 0.03
        self.roll_correction_param = 0 # 0.3
        #self.y_correction_param = 0 #50
        #self.dynamic_tilt_param = 0.04
        self.amp_correction_param = 600 
        tip_Elevation_On = self.stepLength >= 80
        heel_Elevation_On = self.stepLength >= 0

                                                          # FASA 1 (Left support leg)
        if self.stepLengthPlaner_is_ON: print('pitch:', self.body_euler_angle['pitch'])
        xt0, dy0, dy = self.stepLengthPlaner(self.stepLength, self.sideLength, framestep, 0)
        self.ztl = -self.gaitHeight
        xtl0  = self.xtl
        xtr0  = self.xtr
        xtl1 = -xt0
        xtr1 = xt0
        dx0 = (xtl1 - xtl0) * framestep / self.fr2
        dx = (xtr1 - xtr0) * framestep / (self.fr2 - 2 * framestep)
        ddx0 = dx0 - self.dx0_last
        for iii in range(0, frameNumberPerStep, framestep):
            delay_allowance = 2
            cycle_pass = True 
            while cycle_pass:
                cycle_pass = False
                if self.glob.SIMULATION == 2: start1 = self.pyb.millis()
                else: start1 = 0
                #if 2 * framestep < iii <  self.fr2 - 4 * framestep:
                if framestep < iii <  self.fr2 - 2 * framestep:
                    xt0, dy0, dy = self.stepLengthPlaner(self.stepLength, self.sideLength, framestep,0)
                    xtl1 = -xt0
                    xtr1 = xt0
                    if cycle == 0:
                        xtl0 = 0
                        xtr0 = 0
                    else:
                        xtl0 = xt0
                        xtr0 = -xt0
                    dx0 = (xtl1 - self.xtl) * framestep / (self.fr2 - iii)
                    dx = (- self.xtr - self.xtl - dx0  * ( (self.fr2 - iii)/ framestep + 1)) /( (self.fr2 - iii)/ framestep - 1)
                    #print('dx01=', dx01, 'dx0=', dx0, )
                    #print('stepLength1 =', stepLength1)
                #if iii == int(frameNumberPerStep/2):
                #    self.amplitude += self.body_euler_angle['roll'] * self.amp_correction_param
                S = self.amplitude/2 *math.cos(alpha01 * iii )
                self.ytr = -S - self.d10
                self.ytl = -S + self.d10
            
                #self.ztr = -self.gaitHeight
                self.ztl = -self.gaitHeight
                #if iii == int(frameNumberPerStep/2):
                #    if self.body_euler_angle['roll'] < 0 :
                #        for p in range(10):
                #            #print('time:', self.timeElapsed, 'pause right')
                #            self.sim_Progress(0.02)
                #            self.refresh_Orientation() 
                #            if self.body_euler_angle['roll'] > 0.1 or self.body_euler_angle['pitch'] > 0.1: break
                #if iii == self.fr2 - 4 * framestep and self.stepLengthPlaner_is_ON:
                #    #print('self.body_euler_angle['roll'] =', self.body_euler_angle['roll'])
                #    if self.body_euler_angle['roll'] > 0.1 or self.body_euler_angle['pitch'] > 0.1:
                #        for p in range(10):
                #            #print('time:', self.timeElapsed, 'pause right')
                #            self.sim_Progress(0.01)
                #            self.refresh_Orientation() 
                #            if self.body_euler_angle['roll'] < 0.1 or self.body_euler_angle['pitch'] < 0.1: break
                if iii == 0 or iii == frameNumberPerStep - framestep :
                    self.ztr = -self.gaitHeight + self.stepHeight/3
                elif iii == 1 or iii == frameNumberPerStep - framestep * 2:
                    self.ztr = -self.gaitHeight + self.stepHeight / 3 * 2
                else: self.ztr = -self.gaitHeight + self.stepHeight
                if iii==0:
                    #self.xtr = xtr0 + dx0 * (iii / framestep + 1)
                    self.xtr += dx0
                    self.ytr = -84 + dy0 * iii
                elif iii==self.fr2 - framestep:
                    #self.xtr = xtr0 + dx * (self.fr2 / framestep - hovernum) +  dx0 * ( iii - self.fr2 + 7 * framestep)/ framestep
                    self.xtr += dx0
                    self.ytr = -84 + dy0 * 3 * framestep  - dy*(self.fr2 - 3 * framestep)/2 + dy0 * (iii - (self.fr2 - 3 * framestep))
                else:
                    #self.xtr = xtr0 + dx * (iii/framestep - 2) + dx0 * 3
                    if delay_allowance == 2:
                        self.xtr += dx
                    self.ytr = - 84 + dy0 * 3 * framestep - dy*iii/2
                    self.wr = wr_old + (wr_target - wr_old) * (iii)/(self.fr2)
                    self.wl = wl_old + (wl_target - wl_old) * (iii)/(self.fr2)
                #self.xtl = xtl0 + dx0 * (iii/framestep + 1)
                self.xtl += dx0
                #print('dx0 = ', dx0, "self.xtl = ", self.xtl)
                self.ytl = -S + self.d10 + dy0 * iii
                self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
                #self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']
                self.refresh_Orientation()
                if iii == int(frameNumberPerStep/4) * 2 and self.body_euler_angle['roll'] < 0 and delay_allowance:
                    cycle_pass = True
                    delay_allowance -= 1
                    print('Pause')
                    #self.sim_Progress(0.02)
                    self.refresh_Orientation()
                z_correction_from_tilt = math.sin(self.body_euler_angle['pitch']) 
                self.ztr += self.xtr * z_correction_from_tilt
                self.ztl += self.xtl * z_correction_from_tilt
                correct_pitch = (-self.body_euler_angle['pitch'] + math.atan(-dx0 / self.body_tilt_param + self.constant_tilt) + self.dynamic_tilt_param * (-ddx0)) * self.correct_pitch_param
                #self.xr, self.xl = correct_pitch, correct_pitch
                self.xr = self.xl = -dx0 / self.body_tilt_param + self.constant_tilt + correct_pitch + self.dynamic_tilt_param * (-ddx0)
                if abs(self.body_euler_angle['roll']) > 0.1:
                    #self.yr += self.body_euler_angle['roll'] * self.roll_correction_param
                    self.yl += self.body_euler_angle['roll'] * self.roll_correction_param
                self.ytl += self.body_euler_angle['roll'] * self.y_correction_param
                self.ytr -= self.body_euler_angle['roll'] * self.y_correction_param / 4
                if self.ytl <= 15: self.ytl = 15
                print("self.body_euler_angle['yaw']:", self.body_euler_angle['yaw'] )
                #if iii == 0 or iii == frameNumberPerStep - framestep:
                #    print( 'FASA 1, iii =', iii, "self.xtr = ", self.xtr, "self.xtl = ", self.xtl)
                no_heel_up_xtr = self.xtr
                no_heel_up_xtl = self.xtl
                if iii < framestep * 5 and heel_Elevation_On:
                    self.foot_center_2_foot_tip()
                    self.xr = self.xtr/ (self.gaitHeight - 42)
                    self.foot_tip_2_foot_center()
                elif iii >= frameNumberPerStep - framestep * 3  and tip_Elevation_On:
                    self.foot_center_2_heel()
                    self.xr = self.xtr/ (self.gaitHeight - 42)
                    self.heel_2_foot_center()
                if iii > frameNumberPerStep - framestep * 3 and heel_Elevation_On:
                    self.foot_center_2_foot_tip()
                    self.xl = self.xtl/ (self.gaitHeight - 42)
                    self.foot_tip_2_foot_center()
                    #print('self.znl = ', self.znl, 'self.xnl = ', self.xnl)
                elif iii == 0 and tip_Elevation_On:
                    self.foot_center_2_heel()
                    self.xl = self.xtl/ (self.gaitHeight - 42)
                    self.heel_2_foot_center()
                successCode = self.feet_Action(start1)
                self.xtr = no_heel_up_xtr
                self.xtl = no_heel_up_xtl
                #print(iii, self.xtr, self.ytr, self.ztr, self.xr, self.yr, self.zr, self.wr, self.xtl, self.ytl, self.ztl, self.xl, self.yl, self.zl, self.wl, successCode)
        self.dx0_last = dx0

                                        # FASA 2 ( Right support leg)

        self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']   #
        if self.stepLengthPlaner_is_ON: print('pitch:', self.body_euler_angle['pitch'])
        xt0, dy0, dy = self.stepLengthPlaner(secondStepLength, self.sideLength, framestep, 0)
        xtl1  = self.xtl
        xtr1  = self.xtr
        self.ztr = -self.gaitHeight
        if cycle == number_Of_Cycles - 1:
            xtl2 = 0
            xtr2 = 0
        else:
            xtl2 = xt0
            xtr2 = -xt0
        dx0 = (xtr2 - xtr1) * framestep / self.fr2
        dx = - dx0 * (self.fr2 + 2 * framestep)/ (self.fr2 - 2 * framestep)
        ddx0 = dx0 - self.dx0_last
        for iii in range(0, frameNumberPerStep, framestep):
            delay_allowance = 2
            cycle_pass = True 
            while cycle_pass:
                cycle_pass = False
                if self.glob.SIMULATION == 2: start1 = self.pyb.millis()
                else: start1 = 0
                #if 2 * framestep < iii <  self.fr2 - 4 * framestep:
                if framestep < iii <  self.fr2 - 2 * framestep:
                    xt0, dy0, dy = self.stepLengthPlaner(secondStepLength, self.sideLength, framestep, 0)
                    xtl1 = -xt0
                    xtr1 = xt0
                    if cycle == number_Of_Cycles - 1:
                        xtl2 = 0
                        xtr2 = 0
                    else:
                        xtl2 = xt0
                        xtr2 = -xt0
                    dx0 = (xtr2 - self.xtr) * framestep / (self.fr2 - iii)
                    dx = (- self.xtr - self.xtl - dx0  * ( (self.fr2 - iii)/ framestep + 1)) /( (self.fr2 - iii)/ framestep - 1)
                    #print('dx0=', dx0, 'dx=', dx)
                    #print('stepLength1 =', stepLength1)
                #if iii == int(frameNumberPerStep/2):
                #    self.amplitude -= self.body_euler_angle['roll'] * self.amp_correction_param
                S = -self.amplitude/2 *math.cos(alpha01 * iii)
                self.ytr = -S - self.d10
                self.ytl = -S + self.d10
            
                #self.ztl = -self.gaitHeight
                self.ztr = -self.gaitHeight
                #if iii == int(frameNumberPerStep/2):
                #    if self.body_euler_angle['roll'] > 0 :
                #        for p in range(10):
                #            #print('time:', self.timeElapsed, 'pause right')
                #            self.sim_Progress(0.02)
                #            self.refresh_Orientation() 
                #            if self.body_euler_angle['roll'] < 0.1 or self.body_euler_angle['pitch'] > 0.1: break
                #if iii == self.fr2 - 4 * framestep and self.stepLengthPlaner_is_ON:
                #    #print('self.body_euler_angle['roll'] =', self.body_euler_angle['roll'])
                #    if self.body_euler_angle['roll'] > 0 or self.body_euler_angle['pitch'] > 0.1:
                #        for p in range(10):
                #            #print('time:', self.timeElapsed, 'pause left')
                #            self.sim_Progress(0.01)
                #            self.refresh_Orientation() 
                #            if self.body_euler_angle['roll'] < 0.0 or self.body_euler_angle['pitch'] < 0.1: break
                if iii == 0 or iii == frameNumberPerStep - framestep :
                    self.ztl = -self.gaitHeight + self.stepHeight / 3
                elif iii == 1 or iii == frameNumberPerStep - framestep * 2 :
                    self.ztl = -self.gaitHeight + self.stepHeight / 3 * 2
                else: self.ztl = -self.gaitHeight + self.stepHeight
                if cycle == number_Of_Cycles - 1:
                    if iii== (self.fr2 - framestep):
                        self.ztl = -self.gaitHeight
                        self.ytl = S + self.d10
                if iii == 0 :
                    self.xtl += dx0
                    self.ytl = S + self.d10 + dy0 * iii
                elif iii == self.fr2 - framestep :
                    self.xtl += dx0
                    self.ytl = S + 84 + dy0 * 3 * framestep - dy * (self.fr2) + dy0 * (iii - (self.fr2 - 3 * framestep))
                else:
                    if delay_allowance == 2:
                        self.xtl += dx
                    self.ytl = S + 84 + dy0 * 3 * framestep - dy * (iii - 3 * framestep)
                    self.wr = wr_target * (1 - iii/self.fr2 * 2)
                    self.wl = wl_target * (1 - iii/self.fr2 * 2)
                self.xtr += dx0
                self.ytr += dy0
                if self.ytl < 84 : self.ytl = 84
                #print('dx = ', dx, "self.xtl = ", self.xtl)
                self.yr, self.yl = - self.params['SOLE_LANDING_SKEW'], self.params['SOLE_LANDING_SKEW']
                #self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK']
                self.refresh_Orientation()
                if iii == int(frameNumberPerStep/4)*2 and self.body_euler_angle['roll'] > 0 and delay_allowance:
                    cycle_pass = True
                    delay_allowance -= 1
                    print('Pause')
                    #self.sim_Progress(0.02)
                    self.refresh_Orientation()
                z_correction_from_tilt = math.sin(self.body_euler_angle['pitch']) 
                self.ztr += self.xtr * z_correction_from_tilt
                self.ztl += self.xtl * z_correction_from_tilt
                correct_pitch = (-self.body_euler_angle['pitch'] + math.atan(-dx0 / self.body_tilt_param + self.constant_tilt) + self.dynamic_tilt_param * (-ddx0)) * self.correct_pitch_param
                #self.xr, self.xl = correct_pitch, correct_pitch
                self.xr = self.xl = -dx0 / self.body_tilt_param + self.constant_tilt + correct_pitch + self.dynamic_tilt_param * (-ddx0)
                if abs(self.body_euler_angle['roll']) > 0.1:
                    self.yr += self.body_euler_angle['roll'] * self.roll_correction_param
                    #self.yl += self.body_euler_angle['roll'] * self.roll_correction_param
                self.ytr += self.body_euler_angle['roll'] * self.y_correction_param
                self.ytl -= self.body_euler_angle['roll'] * self.y_correction_param / 4
                if self.ytr >= -15: self.ytr = -15
                #if iii == 0 or iii == frameNumberPerStep - framestep:
                #    print( 'FASA 2, iii =', iii, "self.xtr = ", self.xtr, "self.xtl = ", self.xtl)
                no_heel_up_xtr = self.xtr
                no_heel_up_xtl = self.xtl
                if iii < framestep * 5 and heel_Elevation_On:
                    self.foot_center_2_foot_tip()
                    self.xl = self.xtl/ (self.gaitHeight - 42)
                    self.foot_tip_2_foot_center()
                    #print('self.znl = ', self.znl, 'self.xnl = ', self.xnl)
                elif iii >= frameNumberPerStep - framestep * 3 and tip_Elevation_On:
                    self.foot_center_2_heel()
                    self.xl = self.xtl/ (self.gaitHeight - 42)
                    self.heel_2_foot_center()
                if iii > frameNumberPerStep - framestep * 3 and heel_Elevation_On:
                    self.foot_center_2_foot_tip()
                    self.xr = self.xtr/ (self.gaitHeight - 42)
                    self.foot_tip_2_foot_center()
                elif iii == 0 and tip_Elevation_On:
                    self.foot_center_2_heel()
                    self.xr = self.xtr/ (self.gaitHeight - 42)
                    self.heel_2_foot_center()
                successCode = self.feet_Action(start1)
                self.xtr = no_heel_up_xtr
                self.xtl = no_heel_up_xtl
                #print(iii + self.fr2, self.xtr, self.ytr, self.ztr, self.xr, self.yr, self.zr, self.wr, self.xtl, self.ytl, self.ztl, self.xl, self.yl, self.zl, self.wl, successCode)
        self.dx0_last = dx0
        # returning xr, xl, yr, yl to initial value
        self.xr, self.xl, self.yr, self.yl = xr_old, xl_old, yr_old, yl_old
        #self.first_Leg_Is_Right_Leg = tmp1

    def stepLengthPlaner(self, regularStepLength, regularSideLength, framestep, hovernum):
        if self.stepLengthPlaner_is_ON == True:
            stepLength1 = regularStepLength
            if - 0.08 <= self.body_euler_angle['pitch'] < - 0.045:
                #stepLength1 += (-self.body_euler_angle['pitch']) * 2000
                if stepLength1 > 140: stepLength1 = 140
                if stepLength1 < -140: stepLength1 = -140
                print('stepLength1=', stepLength1)
            elif self.body_euler_angle['pitch'] < - 0.1:
                stepLength1 = 140
                print('stepLength1=', stepLength1)
            elif 0.045 < self.body_euler_angle['pitch'] <= 0.1:
                #stepLength1 += (-self.body_euler_angle['pitch']) * 2000
                if stepLength1 > 140: stepLength1 = 140
                if stepLength1 < -140: stepLength1 = -140
                print('stepLength1=', stepLength1)
            elif 0.08 < self.body_euler_angle['pitch'] :
                stepLength1 = -140
                print('stepLength1=', stepLength1)
            xt0 = stepLength1 /2 * self.fr2 / (self.fr2 + framestep * hovernum)
            if self.body_euler_angle['roll'] > 0.2 : 
                sideLength = 60
                print('sideLength = ', sideLength)
            if self.body_euler_angle['roll'] < -0.2 : 
                sideLength = -60
                print('sideLength = ', sideLength)
            else: sideLength = regularSideLength
            dy0 = sideLength / (self.fr2 + hovernum * framestep) * framestep        # CoM propulsion sideways per framestep
            dy = sideLength /(self.fr2 - hovernum * framestep) * framestep
        else:
            xt0 = regularStepLength /2 * self.fr2 / (self.fr2 + framestep * hovernum)
            dy0 = regularSideLength / (self.fr2 + hovernum * framestep) * framestep        # CoM propulsion sideways per framestep
            dy = regularSideLength /(self.fr2 - hovernum * framestep) * framestep
        return xt0, dy0, dy

    def make_foot_vector_normed(self):
        norm_r = math.sqrt(self.xr * self.xr + self.yr * self.yr + self.zr * self.zr)
        norm_l = math.sqrt(self.xl * self.xl + self.yl * self.yl + self.zl * self.zl)
        self.xr = self.xr / norm_r
        self.yr = self.yr / norm_r
        self.zr = self.zr / norm_r
        self.xl = self.xl / norm_l
        self.yl = self.yl / norm_l
        self.zl = self.zl / norm_l

    def foot_tip_2_foot_center(self):
        self.make_foot_vector_normed()
        self.xtr = self.xnr - self.e10 * math.cos(math.asin(self.xr)) * math.cos(self.wr)
        self.ytr = self.ynr - self.e10 * math.sin(self.wr) * math.cos(math.asin(self.yr))
        self.ztr = self.znr - self.e10 * self.xr * math.cos(self.wr)
        self.xtl = self.xnl - self.e10 * math.cos(math.asin(self.xl)) * math.cos(self.wl)
        self.ytl = self.ynl - self.e10 * math.sin(self.wl) * math.cos(math.asin(self.yl))
        self.ztl = self.znl - self.e10 * self.xl * math.cos(self.wl)

    def foot_center_2_foot_tip(self):
        self.make_foot_vector_normed()
        self.xnr = self.xtr + self.e10 * math.cos(math.asin(self.xr)) * math.cos(self.wr)
        self.ynr = self.ytr + self.e10 * math.sin(self.wr) * math.cos(math.asin(self.yr))
        self.znr = self.ztr + self.e10 * self.xr * math.cos(self.wr)
        self.xnl = self.xtl + self.e10 * math.cos(math.asin(self.xl)) * math.cos(self.wl)
        self.ynl = self.ytl + self.e10 * math.sin(self.wl) * math.cos(math.asin(self.yl))
        self.znl = self.ztl + self.e10 * self.xl * math.cos(self.wl)

    def heel_2_foot_center(self):
        self.make_foot_vector_normed()
        self.xtr = self.xpr + self.e10 * math.cos(math.asin(self.xr)) * math.cos(self.wr)
        self.ytr = self.ypr + self.e10 * math.sin(self.wr) * math.cos(math.asin(self.yr))
        self.ztr = self.zpr + self.e10 * self.xr * math.cos(self.wr)
        self.xtl = self.xpl + self.e10 * math.cos(math.asin(self.xl)) * math.cos(self.wl)
        self.ytl = self.ypl + self.e10 * math.sin(self.wl) * math.cos(math.asin(self.yl))
        self.ztl = self.zpl + self.e10 * self.xl * math.cos(self.wl)

    def foot_center_2_heel(self):
        self.make_foot_vector_normed()
        self.xpr = self.xtr - self.e10 * math.cos(math.asin(self.xr)) * math.cos(self.wr)
        self.ypr = self.ytr - self.e10 * math.sin(self.wr) * math.cos(math.asin(self.yr))
        self.zpr = self.ztr - self.e10 * self.xr * math.cos(self.wr)
        self.xpl = self.xtl - self.e10 * math.cos(math.asin(self.xl)) * math.cos(self.wl)
        self.ypl = self.ytl - self.e10 * math.sin(self.wl) * math.cos(math.asin(self.yl))
        self.zpl = self.ztl - self.e10 * self.xl * math.cos(self.wl)

    def feet_Action(self, start1):
        angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1 )
        if not self.falling_Flag ==0: return
        if len(angles)==0:
            self.exitFlag = self.exitFlag +1
            print('No IK result')
            if self.glob.SIMULATION == 2:
                time1 = self.pyb.elapsed_millis(start1)
                self.pyb.delay(self.frame_delay - time1)
            else:
                if self.glob.SIMULATION == 1:
                    self.sim.simxSynchronousTrigger(self.clientID)
                if self.glob.SIMULATION == 3: self.wait_sim_step()
            return False
        else:
            if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                if self.glob.SIMULATION == 3: self.wait_sim_step()
                self.sim.simxPauseCommunication(self.clientID, True)
                for i in range(len(angles)):
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                        returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                    self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                    self.sim.simx_opmode_oneshot)
                    elif self.glob.SIMULATION == 0:
                        returnCode = self.sim.simxSetJointPosition(self.clientID,
                                    self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                    self.sim.simx_opmode_oneshot)
                self.sim.simxPauseCommunication(self.clientID, False)
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    time.sleep(self.slowTime)
                    returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID, self.Dummy_1Handle , -1, self.sim.simx_opmode_buffer)
                    self.Dummy_HData.append(Dummy_Hposition)
                    returnCode, self.Ballposition= self.sim.simxGetObjectPosition(self.clientID, self.BallHandle , -1, self.sim.simx_opmode_buffer)
                    self.BallData.append(self.Ballposition)
                    #self.refresh_Orientation()
                    self.Dummy_1_YawData.append(self.imu_body_yaw())
                    self.Dummy_1_PitchData.append(self.body_euler_angle['pitch'])
                    self.Dummy_1_RollData.append(self.body_euler_angle['roll'])
                    #print(self.euler_angle)
                    self.timeElapsed = self.timeElapsed +1
                    #print(Dummy_Hposition)
                    if self.glob.SIMULATION == 1:
                        self.sim.simxSynchronousTrigger(self.clientID)
                    #disp = []
                    #for i in range(len(angles)):
                    #    pos = int(angles[i]*1698 + 7500)
                    #    #disp.append((self.ACTIVEJOINTS[i],self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos -7500))
                    #    if 0 <= i <= 4 or 11 <= i <= 15:
                    #        disp.append(pos -7500)
                    #print(iii, self.body_euler_angle['roll'])
                    delta_angles = []
                    for an in range(21):
                        delta_angles.append(round((angles[an] - self.tempangles[an])/0.2, 2))
                    #print(delta_angles)
                    report2 = ''
                    position_o = int(self.gaitHeight * math.tan(-self.body_euler_angle['roll']))
                    position_l = self.ytl
                    position_r = self.ytr
                    self.position_o.append(position_o)
                    if self.ztl == -self.gaitHeight:
                        self.position_l.append(position_l)
                    else: self.position_l.append(-1000)
                    if self.ztr == -self.gaitHeight:
                        self.position_r.append(position_r)
                    else: self.position_r.append(1000)
                    self.tempangles = angles
            elif self.glob.SIMULATION == 2:
                servoDatas = []
                disp = []
                for i in range(len(angles)):
                    pos = int(angles[i]*1698 + 7500)
                    disp.append((self.ACTIVEJOINTS[i],self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos -7500))
                    servoDatas.append( self.kondo.ServoData(self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos))
                servoDatas = self.reOrderServoData(servoDatas)
                start2 = self.pyb.millis()
                #self.kondo.com.start1 = self.pyb.millis()
                a=self.kondo.setServoPos (servoDatas, self.frames_per_cycle)
                #a=self.kondo.setKondoMVServoPos(servoDatas, self.frames_per_cycle)
                #print('disp[4] = ', disp[4], 'disp[15]=', disp[15])
                #print('after read time =', self.pyb.elapsed_micros(self.kondo.com.start3))
                time2 = self.pyb.elapsed_millis(start2)
                time1 = self.pyb.elapsed_millis(start1)
                #print('calc time =',time1 - time2, 'transfer time =', time2, 'delay time : ', (self.frame_delay - time1) )
                self.pyb.delay(self.frame_delay - time1)
            return True

    def walk_Final_Pose(self):
        return 
        self.robot_In_0_Pose = False
        if not self.falling_Test() == 0:
            if self.falling_Flag == 3: print('STOP!')
            else: print('FALLING!!!', self.falling_Flag)
            return[]
        xr_old, xl_old, yr_old, yl_old = self.xr, self.xl, self.yr, self.yl
        # correction of body tilt forward
        self.xr, self.xl = self.params['BODY_TILT_AT_WALK'], self.params['BODY_TILT_AT_WALK'] 
        initPoses = self.initPoses * 4
        framestep = self.simThreadCycleInMs//10
        for j in range (initPoses):
            if self.glob.SIMULATION == 2: start1 = self.pyb.millis()
            self.ztr = -self.gaitHeight + (j+1)*(self.ztr0+self.gaitHeight)/initPoses
            self.ztl = -self.gaitHeight + (j+1)*(self.ztl0+self.gaitHeight)/initPoses
            self.ytr = -self.d10 - (initPoses-(j+1))*self.amplitude/2 /initPoses
            self.ytl =  self.d10 - (initPoses-(j+1))*self.amplitude/2 /initPoses
            if j == initPoses - 1:
                angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1, hands_on = False)
            else: angles = self.computeAlphaForWalk(self.SIZES, self.limAlpha1 )
            #if not self.falling_Flag ==0: return
            if len(angles)==0:
                self.exitFlag = self.exitFlag +1
                print('walk_Final_Pose: failure with IK')
            else:
                if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                    if self.glob.SIMULATION == 3: self.wait_sim_step()
                    self.sim.simxPauseCommunication(self.clientID, True)
                    for i in range(len(angles)):
                        if self.keep_hands_up:
                            if i in self.hand_joints : continue
                        if self.glob.SIMULATION == 1 or self.glob.SIMULATION == 3:
                           returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                        elif self.glob.SIMULATION == 0:
                           returnCode = self.sim.simxSetJointPosition(self.clientID,
                                        self.jointHandle[i] , angles[i]*self.FACTOR[i]+self.trims[i],
                                        self.sim.simx_opmode_oneshot)
                    self.sim.simxPauseCommunication(self.clientID, False)
                    if self.glob.SIMULATION == 1 or self.glob.SIMULATION  == 0 or self.glob.SIMULATION == 3:
                        time.sleep(self.slowTime)
                        returnCode, Dummy_Hposition= self.sim.simxGetObjectPosition(self.clientID,
                                              self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
                        #self.Dummy_HData.append(Dummy_Hposition)
                        #self.timeElapsed = self.timeElapsed +1
                        # print(Dummy_Hposition)
                    if self.glob.SIMULATION == 1:
                        self.sim.simxSynchronousTrigger(self.clientID)
                elif self.glob.SIMULATION == 2:
                    servoDatas = []
                    for i in range(len(angles)):
                        if self.keep_hands_up:
                            if i in self.hand_joints : continue
                        pos = int(angles[i]*1698 + 7500)
                        servoDatas.append( self.kondo.ServoData(self.ACTIVESERVOS[i][0],self.ACTIVESERVOS[i][1],pos))
                    servoDatas = self.reOrderServoData(servoDatas)
                    start2 = self.pyb.millis()
                    a=self.kondo.setServoPos (servoDatas, self.frames_per_cycle)
                    #print(servoDatas)
                    #print(clock.avg())
                    time1 = self.pyb.elapsed_millis(start1)
                    time2 = self.pyb.elapsed_millis(start2)
                    if time1 < self.frame_delay: self.pyb.delay(self.frame_delay - time1)
        self.xr, self.xl, self.yr, self.yl = xr_old, xl_old, yr_old, yl_old

    def refresh_Orientation(self):
        if self.glob.SIMULATION == 2:
            head_quaternuon_raw = [0,0,0,0]#self.imu.quaternion()
            head_quaternion = self.quat_multi(head_quaternuon_raw, [0.7071, 0, 0.7071, 0])
            self.euler_angle = self.quaternion_to_euler_angle(head_quaternion)
            self.euler_angle['yaw'] += math.pi/2
            self.euler_angle['roll'] += math.pi/2
            self.head_quaternion_2_body_euler_angle(head_quaternion)
            self.body_euler_angle['yaw'] += math.pi/2
            self.body_euler_angle['roll'] += math.pi/2
        else:
            returnCode, Dummy_Hquaternion= self.sim.simxGetObjectQuaternion(self.clientID, self.Dummy_HHandle , -1, self.sim.simx_opmode_buffer)
            head_quaternion = self.from_vrep_quat_to_conventional_quat(Dummy_Hquaternion)
            self.euler_angle = self.quaternion_to_euler_angle(head_quaternion)
            self.head_quaternion_2_body_euler_angle(head_quaternion)
        self.euler_angle['yaw'] -= self.direction_To_Attack
        self.body_euler_angle['yaw'] -= self.direction_To_Attack
        self.body_euler_angle['roll'] -= self.zero_pozition_body_roll 
        self.body_euler_angle['pitch'] -= self.zero_pozition_body_pitch

    def normalize_rotation(self, yaw):
        if abs(yaw) > 2 * math.pi: yaw %= (2 * math.pi)
        if yaw > math.pi : yaw -= (2 * math.pi)
        if yaw < -math.pi : yaw += (2 * math.pi)
        if yaw > 0.5 : yaw = 0.5
        if yaw < -0.5 : yaw = -0.5
        return yaw

    def from_vrep_quat_to_conventional_quat(self, quat):
        x,y,z,w = quat
        return [w,x,y,z]

    def quat_multi(self, quat1, quat2):
        w1,x1,y1,z1 = quat1
        w2,x2,y2,z2 = quat2
        """
            Quaternion multiplication
            multi = (w1 + x1*i + y1*j + z1*k) * ( w2 + x2*i + y2*j + z2*k) = 
            w1*w2 +x1*w2*i + y1*w2*j + z1*w2*k + w1*x2*i - x1*x2 - y1*x2*k + z1*x2*j +
            w1*y2*j + x1*y2*k - y1*y2 - z1*y2*i + w1*z2*k - x1*z2*j + y1*z2*i - z1*z2 =
            (w1*w2 - x1*x2 - y1*y2 - z1*z2) + (x1*w2 + w1*x2 - z1*y2 + y1*z2)*i +
            (y1*w2 + z1*x2 + w1*y2 - x1*z2)*j + (z1*w2 - y1*x2 + x1*y2 + w1*z2)*k
        """
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = x1*w2 + w1*x2 - z1*y2 + y1*z2
        y = y1*w2 + z1*x2 + w1*y2 - x1*z2
        z = z1*w2 - y1*x2 + x1*y2 + w1*z2
        return [w,x,y,z]

    def quat_division(self, quat1, quat2):
        """
        Quaternion division
        """
        w2,x2,y2,z2 = quat2
        coquat2 = [w2, -x2, -y2, -z2]
        quat = self.quat_multi(quat1, coquat2)
        w,x,y,z = quat
        sqquat2 = w2*w2 + x2*x2 + y2*y2 + z2*z2
        return [w/sqquat2, x/sqquat2, y/sqquat2, z/sqquat2]

    def head_quaternion_2_body_euler_angle(self, head_quaternion):
        tilt_angle = - (self.neck_tilt - self.zero_pozition_neck_tilt) * self.TIK2RAD
        pan_angle = - self.neck_pan * self.TIK2RAD
        tilt_quat = [math.cos(tilt_angle/2), 0, math.sin(tilt_angle/2), 0]
        pan_quat = [math.cos(pan_angle/2), 0, 0, math.sin(pan_angle/2)]
        body_dummy_quaternion = self.quat_division(head_quaternion, tilt_quat)
        body_dummy_quaternion = self.quat_division(body_dummy_quaternion, pan_quat)
        self.body_euler_angle = self.quaternion_to_euler_angle(body_dummy_quaternion)

    def move_head(self, pan, tilt):
        self.neck_pan = pan
        self.neck_tilt = tilt
        if self.glob.SIMULATION == 2:
                self.kondo.setUserParameter(19,self.neck_pan)
                self.pyb.delay(200)
                self.kondo.setUserParameter(20,self.neck_tilt)
                self.pyb.delay(400)
        else:
            returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                        self.jointHandle[21] , self.neck_pan * self.TIK2RAD * self.FACTOR[21], self.sim.simx_opmode_oneshot)   # Шея поворот
            returnCode = self.sim.simxSetJointTargetPosition(self.clientID,
                        self.jointHandle[22] , self.neck_tilt * self.TIK2RAD * self.FACTOR[22], self.sim.simx_opmode_oneshot)  # Шея Наклон
            for j in range(20):
                self.sim.simxSynchronousTrigger(self.clientID)



if __name__=="__main__":
    print('This is not main module!')
