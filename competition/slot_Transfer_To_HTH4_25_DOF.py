"""
module provide input of motion slot from json file to HTH4 in typewriter mode. 
Usage: 
In HTH4 create in new motion file composing layout one "Pos0" control, click mouse one time on control,
then activate program by Hotkey: 'cntrl+shift+1'
"""

import keyboard
import pyautogui as pag
import time
import json
import sys
pag.FAILSAFE = False


slot_name = 'Ball_tennis_throw_v7_2'
motion_slot_filename = 'C:/Users/ab/Documents/GitHub/Robokit/FIRA_2021_2/Soccer/Motion/motion_slots/' + slot_name + '.json'
with open(motion_slot_filename, "r") as f:
    loaded_Dict = json.loads(f.read())
motion_set = loaded_Dict[slot_name]

interval = 0.01         # time between keyboard characters input
global slide_direction
slide_direction = 1

def create_pose_page(number):
    global slide_direction
    pag.click(button='left')
    if pag.position()[0] > 2000 and slide_direction == 1:
        slide_direction = -1
        pag.move(160, 120)
    if pag.position()[0] < 200 and slide_direction == -1:
        slide_direction = 1
        pag.move(-160, 120)
    if slide_direction == 1:
        pag.move(80, -50)
    else:
        pag.move(-240, -50)
    pag.hotkey('ctrl', 'c')
    pag.hotkey('ctrl', 'v')
    pag.hotkey('ctrl', 'r')
    pag.typewrite (['backspace'], interval = interval)
    pose_name = 'Pos'+ str(number)
    pag.typewrite (pose_name, interval = interval)
    pag.typewrite (['enter'], interval = interval)

control_sequence = [22, 11, 21, 10, 20, 9, 19, 8, 18, 7, 17, 6, 16, 5, 15, 4, 14, 3, 13, 2, 12, 1, 25, 24, 23] # order of controls by tab key in HTH4

def key_sim():
    page_number = len(motion_set)
    for page in range(page_number):
        if page != 0:
            create_pose_page(page)
            pag.move(80, 50)
        pag.doubleClick()
        time.sleep(1)
        for i in range(5):
            pag.typewrite (['del'], interval = interval)
        str_value = str(motion_set[page][0])
        pag.typewrite (str_value, interval = interval)
        for i in range(3):
            pag.typewrite (['tab'], interval = interval)
        for turn in control_sequence:
            for i in range(5):
                pag.typewrite (['del'], interval = interval)
            str_value = str(motion_set[page][turn])
            pag.typewrite (str_value, interval = interval)
            for i in range(2):
                pag.typewrite (['tab'], interval = interval)
        pag.typewrite (['tab'], interval = interval)
        pag.typewrite (['enter'], interval = interval)
    sys.exit(0)
        
keyboard.add_hotkey('ctrl+shift+1', key_sim)

while True:
    time.sleep(60)