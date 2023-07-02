# write from player.py
from competition import Competition

class RunTurfTest(Competition):
    def __init__(self, button):
        super().__init__(button)
        # st else

    def run(self):
        if self.button == 2 or self.button ==3 :
            self.sidestep_test_main_cycle(self.button)
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

if __name__ == '__main__':
    default_button = 1
    turf_tester = RunTurfTest(default_button)
    print("running the run_turf_test.py")
    turf_tester.run()
    print("end of running the run_turf_test.py")