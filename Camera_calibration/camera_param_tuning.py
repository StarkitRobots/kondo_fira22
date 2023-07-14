import cv2
import numpy as np
import os, sys
os.chdir('../')
directory_for_KondoCameraSensor = os.getcwd()
sys.path.append(directory_for_KondoCameraSensor + '/competition/Soccer/Motion/')
print(sys.path)
from reload import KondoCameraSensor
#import arducam_mipicamera
#import v4l2
#sudo pip install v4l2
# cam = cv2.VideoCapture(0)

low_th  = [0, 0, 0]
high_th = [255, 255, 255]

def nothing (x):
    pass

cv2.namedWindow ('Colorbars')

cv2.createTrackbar ("l1", "Colorbars",   0, 255, nothing)
cv2.createTrackbar ("h1", "Colorbars", 255, 255, nothing)
cv2.createTrackbar ("l2", "Colorbars",   0, 255, nothing)
cv2.createTrackbar ("h2", "Colorbars", 255, 255, nothing)
cv2.createTrackbar ("l3", "Colorbars",   0, 255, nothing)
cv2.createTrackbar ("h3", "Colorbars", 255, 255, nothing)
sensor = KondoCameraSensor("~/Desktop/kondo_fira22/Camera_calibration/mtx.yaml")
# frm = cv2.imread('photo1.png')


while (True):    
    # _, frame = cam.read()
    frm = sensor.snapshot().img
    width = int(frm.shape[1] * 0.45)
    height = int(frm.shape[0] * 0.45)
    dim = (width, height)
    frame = cv2.resize(frm, dim, interpolation = cv2.INTER_AREA)
    l1 = cv2.getTrackbarPos ("l1", "Colorbars")
    h1 = cv2.getTrackbarPos ("h1", "Colorbars")
    l2 = cv2.getTrackbarPos ("l2", "Colorbars")
    h2 = cv2.getTrackbarPos ("h2", "Colorbars")
    l3 = cv2.getTrackbarPos ("l3", "Colorbars")
    h3 = cv2.getTrackbarPos ("h3", "Colorbars")

    low_th  = (l1, l2, l3)
    high_th = (h1, h2, h3)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, tuple(low_th), tuple(high_th))

    mask_3_channels = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    result = np.concatenate((frame, mask_3_channels), axis = 1)
    cv2.imshow ("Colorbars", result)

    #os.system ('clear')

    print (low_th, high_th)

    key = cv2.waitKey(150) & 0xFF

    if (key == ord('q')):
        break
		
cam.release()
cv2.destroyAllWindows()
