import arducam_mipicamera as arducam
import v4l2 #sudo pip install v4l2
import time
import cv2 #sudo apt-get install python-opencv
import numpy as np
import yaml

def align_down(size, align):
    return (size & ~((align)-1))

def align_up(size, align):
    return align_down(size + align - 1, align)

def set_controls(camera):
    try:
        print("Reset the focus...")
        camera.reset_control(v4l2.V4L2_CID_FOCUS_ABSOLUTE)
    except Exception as e:
        print(e)
        print("The camera may not support this control.")

    try:
        # iprint("Enable Auto Exposure...")
        camera.software_auto_exposure(enable = True)
        print("Enable Auto White Balance...")

        camera.software_auto_white_balance(enable = True)
        # camera.manual_set_awb_compensation(100, 100)

        cv2.waitKey(1000)

    except Exception as e:
        print(e)
def loadCoefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    camera_matrix = cv_file.getNode("camera_matrix").mat()
    dist_matrix = cv_file.getNode("dist_coeff").mat()
    cv_file.release()
    return [camera_matrix, dist_matrix]
if __name__ == "__main__":
    try:
        #sudo apt install v4l-utils


       #set_flags(flags) 
        camera = arducam.mipi_camera()
        print("Open camera...")
        camera.init_camera()
        camera.set_mode(6)
        '''Available mode:
       mode: 0, width: 1600, height: 1300, pixelformat: GREY, desc: (null)
       mode: 1, width: 1600, height: 1300, pixelformat: BA81, desc: color mode
       mode: 2, width: 1600, height: 1080, pixelformat: GREY, desc: (null)
       mode: 3, width: 1600, height: 1080, pixelformat: BA81, desc: color mode
       mode: 4, width: 1920, height: 1080, pixelformat: GREY, desc: (null)
       mode: 5, width: 1920, height: 1080, pixelformat: BA81, desc: color mode
       mode: 6, width: 1600, height: 1300, pixelformat: Y10P, desc: 1600x1300 RAW 10 mode 
       mode: 7, width: 1600, height: 1300, pixelformat: GREY, desc: Used for ov2311 2lane raw8 1600x1300 external trigger mode
       mode: 8, width: 1600, height: 1080, pixelformat: GREY, desc: Used for ov2311 2lane raw8 1600x1080 external trigger mode
       mode: 9, width: 1920, height: 1080, pixelformat: GREY, desc: Used for ov2311 2lane raw8 1920x1080 external trigger mode
       mode: 10, width: 1600, height: 1300, pixelformat: Y10P, desc: Used for ov2311 2lane raw10 1600x1300 external trigger mode
        '''


        print("Setting the resolution...")
        fmt = camera.set_resolution(1600, 1300)
        # fmt = camera.set_resolution(480, 360)
        # print("Current resolution is {}".format(fmt))
        # set_controls(camera)
        mtx = []
        dist = []
        with open("calibration_matrix.yaml", 'r') as f:
            data = yaml.load(f, Loader=yaml.loader.UnsafeLoader)
            mtx = np.array([np.array(i) for i in data["camera_matrix"]])        
            dist = np.array( data["dist_coeff"][0])
            print (mtx,dist,sep="\n")

            # Debug: print the values
        mtx, dist = loadCoefficients("mtx.yaml")
        while cv2.waitKey(10) != 27:
            import time
            time_now = time.time()
            frame = camera.capture(encoding = 'raw') #i420
            # fr = np.array(frame)
            # print("original shape", fr.shape, fr)
            height = int(align_up(1300, 16))
            width = int(align_up(1600, 32))
            image = frame.as_array.reshape(int(height), width) # * 1.5
            image = cv2.cvtColor(image, cv2.COLOR_BayerRG2BGR) # BG
            # image = cv2.imdecode(frame.as_array,cv2.IMREAD_UNCHANGED)
            # image = cv2.resize(image,(720, 540), interpolation=cv2.INTER_AREA)
            # image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            # image[:][:][1], image[:][:][2] = image[:][:][2], image[:][:][1]
            (h, w, d) = image.shape 
            center = (w // 2, h // 2)
            M = cv2.getRotationMatrix2D(center, 180, 1.0)
            rotated = cv2.warpAffine(image, M, (w, h))  
            #img2 = cv2.cvtColor(image, cv2.COLOR_BayerGR2BGR) # BG
            #img3 = cv2.cvtColor(image, cv2.COLOR_BayerBG2BGR) # BG
            #img4 = cv2.cvtColor(image, cv2.COLOR_BayerGB2BGR) # BG
            
            #im1 = np.concatenate((img1, img2), axis=1)
            #im2 = np.concatenate((img3, img4), axis=1)
            
            #image_ = np.concatenate((im1, im2), axis=0)
            image = rotated
            #image = cv2.resize(image_, (1920, 1080))
            print(image.shape, width, height)
            arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            arucoParams = cv2.aruco.DetectorParameters_create()
            (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
                        parameters=arucoParams)
            # verify *at least* one ArUco marker was detected

            if len(corners) > 0:
                    # flatten the ArUco IDs list
                    ids = ids.flatten()
                    # loop over the detected ArUCo corners
                    a = np.eye(3)
                    tvec, rvec = cv2.aruco.estimatePoseSingleMarkers(corners[0], 0.05, mtx, dist)
                    print("rvec : ", rvec)
                    print("tvec : ", tvec)
                    for (markerCorner, markerID) in zip(corners, ids):
                        # extract the marker corners (which are always returned in
                        # top-left, top-right, bottom-right, and bottom-left order)
                        corners = markerCorner.reshape((4, 2))
                        (topLeft, topRight, bottomRight, bottomLeft) = corners
                        # convert each of the (x, y)-coordinate pairs to integers
                        topRight = (int(topRight[0]), int(topRight[1]))
                        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                        topLeft = (int(topLeft[0]), int(topLeft[1]))
                        # draw the bounding box of the ArUCo detection
                        cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                        cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                        cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                        cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                        # compute and draw the center (x, y)-coordinates of the ArUco
                        # marker
                        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                        cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                        # draw the ArUco marker ID on the image
                        cv2.putText(image, str(markerID),
                        (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
                        print("[INFO] ArUco marker ID: {}".format(markerID))
                        # show the output image
            # image =cv2.transpose(cv2.transpose(image))
            image_und = cv2.undistort(
            image, mtx, dist, None, None
            )
            im2 = np.concatenate((image, image_und), axis=1)
            cv2.imshow("Arducam", im2)
            print(1/(time.time() - time_now))

        # Release memory
        del frame
        print("Close camera...")
        camera.close_camera()
    except Exception as e:
        print(e)
