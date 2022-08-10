import arducam_mipicamera as arducam
import v4l2 #sudo pip install v4l2
import time
import numpy as np
import cv2 #sudo apt-get install python-opencv
import os
import yaml

def set_controls(camera):
    try:
        #print("Enable Auto Exposure...")
        #camera.software_auto_exposure(enable = True)
        camera.set_control(v4l2.V4L2_CID_EXPOSURE, 800)  # 0 < 65535
        camera.set_control(v4l2.V4L2_CID_GAIN, 255)     # 0 < 255
        camera.set_control(v4l2.V4L2_CID_VFLIP, 1)
        print('exposure:', camera.get_control(v4l2.V4L2_CID_EXPOSURE), 'gain: ', camera.get_control(v4l2.V4L2_CID_GAIN))
        #print("Enable Auto White Balance...")
        #camera.software_auto_white_balance(enable = True)
    except Exception as e:
        print(e)
camera = arducam.mipi_camera()
print("Open camera...")
camera.init_camera()
camera.set_mode(6) # chose a camera mode which yields raw10 pixel format, see output of list_format utility
fmt = camera.get_format()
width = fmt.get("width")
height = fmt.get("height")
print("Current resolution is {w}x{h}".format(w=width, h=height))
set_controls(camera)
with open("/home/pi/MIPI_Camera_old/RPI/python/Camera_calibration/calibration_matrix.yaml", "r")as f:
    data = yaml.load(f, yaml.Loader)
camera_matrix = np.asarray(data['camera_matrix'])
dist_coeff = np.asarray(data['dist_coeff'])
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeff , (width,height), 1, (width,height))
roi_x, roi_y, roi_w, roi_h = roi
map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeff, None, newcameramtx, (width,height), cv2.CV_16SC2)
time.sleep(0.5)
timer = time.perf_counter()
while(True):
    image = camera.capture(encoding = 'raw')
    image = arducam.remove_padding(image.data, width, height, 10)
    image = arducam.unpack_mipi_raw10(image)
    image = image.reshape(height, width) << 6
    #print(image.size, image.dtype, image.shape)
    image = cv2.cvtColor(image, cv2.COLOR_BayerRG2BGR)
    image = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)
    cv2_image = image[roi_y : roi_y + roi_h, roi_x : roi_x + roi_w]
    cv2_image = cv2.resize(cv2_image, (480,360), interpolation= cv2.INTER_LINEAR)
    cv2_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2RGB)
    cv2.imshow("Arducam", cv2_image)
    cv2.waitKey(10)
    print('FPS:', 1/(time.perf_counter() - timer))
    timer = time.perf_counter()
