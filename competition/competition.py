
import pyb
import os
import sys
import json

current_work_directory = os.getcwd()
current_work_directory += '/'
sys.path.append( current_work_directory )
sys.path.append( current_work_directory + 'Soccer/')
sys.path.append( current_work_directory + 'Soccer/Motion/')
SIMULATION=2
# from class_Motion import Glob
if SIMULATION == 2:
    from class_Motion import Motion1 as Motion
    from class_Motion import Glob
    from reload import KondoCameraSensor
class Competition():
    def __init__(self, button):
        # SIMULATION = 2
        self.glob = Glob(SIMULATION, current_work_directory)
        # self.motion = None
        self.dl_params ={}
        self.motion = Motion(self.glob)
        self.sensor = KondoCameraSensor("/home/pi/kondo_fira22/Camera_calibration/mtx.yaml") # dirty hack 
        self.button = button
        self.common_init()

        with open(current_work_directory + "Soccer/Init_params/Real/Real_walk_params.json", "r", "r") as f:
                self.params = json.loads(f.read())
        self.number_of_cycles = self.params["number_of_cycles"]
        self.motion.simThreadCycleInMs = self.params["simThreadCycleInMs"]
        self.motion.amplitude = self.params["amplitude"]
        self.motion.fr1 = self.params["fr1"]
        self.motion.fr2 = self.params["fr2"]
        self.motion.initPoses = self.motion.fr2 
        self.motion.gaitHeight = self.params["gaitHeight"]
        self.motion.stepHeight = self.params["stepHeight"]
        self.stepLength = self.params["stepLength"]
        self.forward = True
        self.stopDistance = self.params["stopDistance"]
        self.stepIncrement = self.params["stepIncrement"]
        self.stepDecrement = self.params["stepDecrement"]
        self.maxStepLengthBack = self.params["maxStepLengthBack"]

    def common_init(self):
        self.motion.activation()
        self.motion.falling_Flag = 0
    
    def sidestep_test_main_cycle(self):
        # write it
        print("in sidestep_test_main_cycle")

if __name__ == '__main__':
    default_button = 1
    competitor = Competition(default_button)