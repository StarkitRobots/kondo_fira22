# write from player.py

from competition import Competition

class RunTest(Competition):
    def __init__(self, button):
        super().__init__(button)
        # st else

    def run(self):
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
        if self.button == 1:             # fast step
            number_Of_Cycles = 30 #30
            self.motion.simThreadCycleInMs = 10
            self.motion.amplitude = 32 #32
            self.motion.fr1 = 4 # 4
            self.motion.fr2 = 9 # 10
            ##self.motion.initPoses = self.motion.fr2 
            self.motion.gaitHeight = 170 # 190
            self.motion.stepHeight = 30  # 20
            stepLength = 50 #88
        if self.button == 2  :   
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

if __name__ == '__main__':
    default_button = 1
    run_tester = RunTest(default_button)
    print("running the run_test.py")
    run_tester.run()
    print("end of running the run_test.py")