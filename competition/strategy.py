


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
    SIMULATION = 2                                         # 0 - Simulation without physics, 1 - Simulation synchronous with physics, 2 - live on openMV
else:
    # will be running on desktop computer
    current_work_directory += '/'
    import threading
    SIMULATION = 1                                          # 3 - Simulation streaming with physics, 4 - simulation webots

sys.path.append( current_work_directory + 'Soccer/')
sys.path.append( current_work_directory + 'Soccer/Motion/')
SIMULATION=2

if SIMULATION == 2:
    from player import Player




if __name__=="__main__":
    player = Player('run_test')  # 'run_test', 'balancing_test', 'basketball', 'weight_lifting', 'kondo_walk'
    #player.deep_learning()
    #for i in range(1):
    #    player.test_walk()
    #if SIMULATION != 0:
    #    player.motion.print_Diagnostics()
    #player.test_walk_2()
    player.real(2)





