import sys
import os
import math
import json
import time
import cv2

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')

if sys.version == '3.4.0':
    # will be running on openMV
    import pyb
    SIMULATION = 2                                         # 0 - Simulation without physics, 1 - Simulation synchronous with physics, 2 - live on openMV
else:
    # will be running on desktop computer
    current_work_directory += '/'
    import threading
    SIMULATION = 1 

sys.path.append( current_work_directory + 'Soccer/')
sys.path.append( current_work_directory + 'Soccer/Motion/')
sys.path.append( current_work_directory + 'Soccer/Motion/Jump/')
sys.path.append( current_work_directory + 'Soccer/Motion/motion_slots')

from approach import approach
from center_point import center_point
from line_detect import line_detect
from localisation import localisation

from reload import Image 
from reload import Blob

r_x, r_y, r_theta = 3, 4, 0 #robot's coordinates
point = [100, 100, 100]
acc = 0.001

while math.fabs(r_x - point[0]) >= acc or math.fabs(r_y - point[1]) >= acc or math.fabs(r_theta - point[2]) >= acc:
    img = Image(cv2.imread("Soccer\\5837.png", cv2.COLOR_BGR2LAB))
    blob = img.find_blobs([[100, 110, 140, 210, 220, 230], [50, 70, 80, 230, 240, 250], [10, 20, 30, 251, 252, 253]], 30, 30)
    print(type(blob[0]))
    r_x = blob[0].x
    r_y = blob[0].y
    if line_detect() == True:
        point = center_point()
        steps = approach(r_x, r_y, r_theta, point[0], point[1], point[2])
        try:
            r_x, r_y, r_theta = localisation(steps[1])
            print(r_x, r_y, r_theta)
        except IndexError:
            r_x, r_y, r_theta = point[0], point[1], point[2]
    if line_detect() == False:
        print(line_detect())
        break

print('im on the line, jump')

