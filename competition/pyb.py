import time
def delay(time_to_sleep):
    if time_to_sleep <= 0:
        return
    time.sleep(time_to_sleep/1000)
def UART(*args):
    print("I am chill UART function from pyb.py and I am do nothing")
def millis():
    return time.time() / 1000
def elapsed_millis(start):
    return millis()-start
