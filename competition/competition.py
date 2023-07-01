
import pyb
import os
import sys
current_work_directory = os.getcwd()
sys.path.append( current_work_directory + 'Soccer/')
sys.path.append( current_work_directory + 'Soccer/Motion/')
SIMULATION=2
from class_Motion import Glob
if SIMULATION == 2:
    from class_Motion import Motion1 as Motion
class Competition():
    def __init__(self, button):
        # SIMULATION = 2
        self.glob = Glob(SIMULATION, current_work_directory)
        self.motion = None
        self.dl_params ={}
        self.motion = Motion(self.glob)
        self.button = button
        self.common_init()

    def common_init(self):
        self.motion.activation()
        self.motion.falling_Flag = 0
    
    def sidestep_test_main_cycle(self):
        # write it
        print("in sidestep_test_main_cycle")

if __name__ == '__main__':
    competitor = Competition()