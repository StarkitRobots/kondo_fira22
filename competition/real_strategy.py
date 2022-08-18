

import sys
import os
import math
import json
import time
SIMULATION=2

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
from player import Player

if 1:
    # will be running on openMV
    import pyb
    SIMULATION = 2                                         # 0 - Simulation without physics, 1 - Simulation synchronous with physics, 2 - live on openMV
    current_work_directory += '/'
else:
    # will be running on desktop computer
    import threading
    SIMULATION = 1                                          # 3 - Simulation streaming with physics, 4 - simulation webots

if __name__=="__main__":
    player = Player('run_test')  # 'run_test', 'balancing_test', 'basketball', 'weight_lifting', 'kondo_walk'
    #player.deep_learning()
    #for i in range(1):
    #    player.test_walk()
    #if SIMULATION != 0:
    #    player.motion.print_Diagnostics()
    #player.test_walk_2()
    player.real(1)





