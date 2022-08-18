import time
import sys
import math

try:
    from button_test import Button_Test
    button = Button_Test()
    pressed_button = button.wait_for_button_pressing()

    from strategy import Player

    if pressed_button == 1:
        player = Player('run_test')
        print('run_test')
        player.real(button)
    if pressed_button == 4:
        player = Player('run_turf_test')
        print('run_turf_test')
        player.real(button)

    if pressed_button == 2:
        player = Player('triple_jump')
        print('triple_jump')
        player.real(button)
    if pressed_button == 3:
        player = Player('weight_lifting')
        print('weight_lifting')
        player.real(button)

except Exception as e:
    f = open("Error.txt",'w')
    sys.print_exception(e,f)
    f.close()
    sys.print_exception(e,sys.stdout)
