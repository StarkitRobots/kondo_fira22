from abc import ABCMeta, abstractmethod, abstractproperty
import cv2
import math
import time
import numpy as np
try:
    import arducam_mipicamera as arducam
    import v4l2
except:
    print ("Cannot find arducam_mipicamera library or v4l2")
class Blob:
    def __init__ (self, x_, y_, w_, h_):
        self.x_ = x_
        self.y_ = y_
        self.w_ = w_
        self.h_ = h_

    def x (self):
        return self.x_

    def y (self):
        return self.y_

    def w (self):
        return self.w_

    def h (self):
        return self.h_

    def cx (self):
        return int (self.x_ + self.w_ / 2)

    def cy (self):
        return int (self.y_ + self.h_ / 2)

    def rect (self):
        return (self.x_, self.y_, self.w_, self.h_)

class Line:
    def __init__ (self, x1_, y1_, x2_, y2_, theta_):
        self.x1_ = x1_
        self.y1_ = y1_
        self.x2_ = x2_
        self.y2_ = y2_
        self.theta_ = theta_

    def x1 (self):
        return self.x1_

    def y1 (self):
        return self.y1_

    def x2 (self):
        return self.x2_

    def y2 (self):
        return self.y2_

    def theta (self):
        return self.theta_

    def line (self):
        return (self.x1_, self.y1_, self.x2_, self.y2_)

class Image:
    def __init__ (self, img_):
        self.img = img_.copy ()

    def find_target_center(self, thblue, thyellow, thred):
        # cv2.imshow("real", frame)
        avarage_x = 0
        avarage_y = 0
        
    #Switch to HSV 
        hsv = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        #cv2.imshow('hsv', cv2.resize(hsv, (320, 260)))
        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        
        low_th_blue = (thblue[0], thblue[2], thblue[4])
        high_th_blue = (thblue[1], thblue[3], thblue[5])

        low_th_yellow = (thyellow[0], thyellow[2], thyellow[4])
        high_th_yellow = (thyellow[1], thyellow[3], thyellow[5])

        low_th_red = (thred[0], thred[2], thred[4])
        high_th_red = (thred[1], thred[3], thred[5])

    #Detect blue circle
        mask_blue = cv2.inRange(hsv, low_th_blue, high_th_blue)    
        

        blured_blue = cv2.medianBlur(mask_blue*gray, 5)
        
        kernel_blue = np.ones((5,5), np.uint8)
        img_erosion_blue = cv2.erode(blured_blue, kernel_blue, iterations=1)
        img_dilation_blue = cv2.dilate(img_erosion_blue, kernel_blue, iterations=1)
        
        canny_blue = cv2.Canny(img_dilation_blue, 350, 360)
        
        #cv2.imshow("blue", cv2.resize(blured_blue, (320, 260)))
        
        circle_blue = cv2.HoughCircles(canny_blue, cv2.HOUGH_GRADIENT, 1.4, 100)
        #cv2.imshow('blue', circle_blue)
        #cv2.waitKey(10)
    #Detect yellow circle
        mask_yellow = cv2.inRange(hsv, low_th_yellow, high_th_yellow)

        
        blured_yellow = cv2.medianBlur(mask_yellow*gray, 5)
        
        kernel_yellow = np.ones((5,5), np.uint8)
        img_erosion_yellow = cv2.erode(blured_yellow, kernel_yellow, iterations=1)
        img_dilation_yellow = cv2.dilate(img_erosion_yellow, kernel_yellow, iterations=1)
        
        canny_yellow = cv2.Canny(img_dilation_yellow, 180, 190)
        
        # cv2.imshow("yellow", canny_yellow)
        
        circle_yellow = cv2.HoughCircles(canny_yellow, cv2.HOUGH_GRADIENT, 1.4, 100)
        
        #cv2.imshow('yellow', cv2.resize(circle_yellow, (320, 260)))
        
        #cv2.waitKey(10)
    #Detect red circle
        mask_red = cv2.inRange(hsv, low_th_red, high_th_red)
        
        cv2.imshow('red', cv2.resize(mask_red, (320, 260)))
        
        blured_red = cv2.medianBlur(mask_red*gray, 7)
        
        kernel_red = np.ones((7,7), np.uint8)
        img_erosion_red = cv2.erode(blured_red, kernel_red, iterations=1)
        img_dilation_red = cv2.dilate(img_erosion_red, kernel_red, iterations=1)
        
        canny_red = cv2.Canny(img_dilation_red, 350, 360) 
        
        # cv2.imshow("red", canny_red)
        
        circle_red = cv2.HoughCircles(canny_red, cv2.HOUGH_GRADIENT, 2.2, 100)

        result = self.img
        # if circle_red is not None:        
        #     circles = np.uint16(np.around(circle_red))
        #     for i in circles[0,:]:
        #         # draw the outer circle
        #         cv2.circle(result,(i[0],i[1]),i[2],(0,255,0),2)
        #         # draw the center of the circle
        #         cv2.circle(result,(i[0],i[1]),2,(0,0,255),3)
        # cv2.imshow('red', cv2.resize(result, (320, 260)))
        
        #cv2.imshow('red', cv2.resize(circle_red, (320, 260)))
    
        #cv2.waitKey(10)
    #Find center
        
    #if circle_blue is not None and circle_yellow is not None and circle_red is not None:
        #convert the (x, y) coordinates and radius of the circles to integers
        counter = 0
        total_x = 0
        total_y = 0
        
        if circle_blue is not None:
            circle_blue = np.round(circle_blue[0, :]).astype("int")
            total_x += circle_blue[0][0]
            total_y += circle_blue[0][1]
            counter += 1
            
        if circle_yellow is not None:
            circle_yellow = np.round(circle_yellow[0, :]).astype("int")
            total_x += circle_yellow[0][0]
            total_y += circle_yellow[0][1]
            counter += 1
            
        if circle_red is not None:
            circle_red = np.round(circle_red[0, :]).astype("int")
            total_x += circle_red[0][0]
            total_y += circle_red[0][1]
            counter += 1
            
        if counter > 0:
            avarage_x = total_x // counter
            avarage_y = total_y // counter
            print(avarage_x, avarage_y)
            # draw a rectangle
            # corresponding to the center of the circles
            cv2.rectangle(result, (avarage_x - 5, avarage_y - 5), (avarage_x + 5, avarage_y + 5), (0, 128, 255), -1)
            # show the output image
        cv2.imshow("output", cv2.resize(result, (320, 260)))
        cv2.waitKey(10)
        return avarage_x , avarage_y


    def find_blobs (self, ths, pixels_threshold, area_threshold, merge=False, margin=0, invert = False):
        masks = []
        for th in ths:
            #low_th = (int(th[0] * 2.55), th[2] + 128, th[4] + 128)
            #high_th = (math.ceil(th[1] * 2.55), th[3] + 128, th[5] + 128)

            low_th = (th[0], th[2], th[4])
            high_th = (th[1], th[3], th[5])

            labimg = cv2.cvtColor (self.img, cv2.COLOR_BGR2HSV)     #color prostranstvo

            mask = cv2.inRange (labimg, low_th, high_th)

            if (invert == True):
                mask = cv2.bitwise_not (mask)

            masks.append (mask)

        cv2.imshow ("maska", mask)
        
        cv2.waitKey(10)
        
        final_mask = masks [0].copy ()
        if (len (masks) > 1):
            for m in masks [1:]:
                final_mask = cv2.bitwise_and (final_mask, m)

        cv2.imshow ("final_maska", final_mask)
        
        cv2.waitKey(10)

        output = cv2.connectedComponentsWithStats (final_mask, 8, cv2.CV_32S)

        #labels_count = output      [0]
        #labels       = output      [1]
        stats        = output      [2]
        #labels_count, labels, stats = output[:3]
        sz = stats.shape[0]

        blobs = []

        for label_num in range (1, sz):
            area = stats [label_num, cv2.CC_STAT_AREA]
            
            if (area >= pixels_threshold):
                new_blob = Blob (stats [label_num, cv2.CC_STAT_LEFT],
                                 stats [label_num, cv2.CC_STAT_TOP],
                                 stats [label_num, cv2.CC_STAT_WIDTH],
                                 stats [label_num, cv2.CC_STAT_HEIGHT])
                up_left_corner = (stats[label_num][0], stats[label_num][1])
                down_right_corner_x = stats[label_num][0] + stats[label_num][2]
                down_right_corner_y = stats[label_num][1] + stats[label_num][3]
                down_right_corner = (down_right_corner_x, down_right_corner_y)
                color_of_rect = (255, 0, 0)
                img_with_labels = cv2.rectangle(final_mask, up_left_corner, down_right_corner, color_of_rect)
                cv2.imshow('labels', img_with_labels)
                #print ("append", area)
                blobs.append (new_blob)

        return blobs

    def binary (self, th):
        low  = (th [0], th [2], th [4])
        high = (th [1], th [3], th [5])
        
        #low = (int(th[0] * 2.55), th[2] + 128, th[4] + 128)
        #high = (math.ceil(th[1] * 2.55), th[3] + 128, th[5] + 128)
        
        labimg = cv2.cvtColor (self.img, cv2.COLOR_RGB2HSV)
        
        mask = cv2.inRange(labimg, low, high)

        sh = mask.shape

        result = np.zeros((sh[0], sh[1], 3), dtype=np.uint8)

        for i in range(0, 3):
            result[:, :, i] = mask.copy()

        return result

    def find_line_segments (self):
        # - Почему Колумб приплыл в Америку, а не в Индию?
        # - Потому что он плавал по одометрии

        #gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(self.img, 50, 150, apertureSize=3)

        #cv2.imshow ("a", edges)

        lines = cv2.HoughLinesP(edges, rho=1, theta = np.pi / 20, threshold = 50, minLineLength =40, maxLineGap =30)
        #lines = cv2.HoughLinesP(edges, rho=1, theta = np.pi / 20, threshold = 10, minLineLength =40, maxLineGap =30)

        resultant_lines = []

        #print (lines)
        try:
            for line in lines:
                x1, y1, x2, y2 = line [0]
                if x1 == x2: theta = 0
                else: theta = math.atan ((y2 - y1) / (x2 - x1))

                new_line = Line (x1, y1, x2, y2, theta)

                resultant_lines.append (new_line)
        except Exception: pass

        return resultant_lines

    def find_lines (self):
        # - Почему Колумб приплыл в Америку, а не в Индию?
        # - Потому что он плавал по одометрии

        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)

        #cv2.imshow ("a", edges)

        lines = cv2.HoughLines(edges, rho=2, theta = np.pi / 6, threshold = 20)

        resultant_lines = []
        #print (lines)
        try:
            for line in lines:
                #print(line)
                rho, theta = line[0][0], line[0][1]
                if math.sin(theta) == 0: x1, x2, y1, y2 = rho, rho, 0, 240
                elif math.cos(theta) == 0: x1, x2, y1, y2 = 0, 320, rho, rho
                else:
                    ind = []
                    y = int(rho/math.sin(theta))
                    if 0 <= y <= 240 : 
                        x = 0
                        ind.append([x,y])
                    y = int((rho - 320* math.cos(theta))/math.sin(theta))
                    if 0 <= y <= 240: 
                        x = 320
                        ind.append([x,y])
                    x = int(rho/math.cos(theta))
                    if 0 <= x <= 320:
                        y = 0
                        ind.append([x,y])
                    x = int((rho - 240 * math.sin(theta))/math.cos(theta))
                    if 0 <= x <= 320:
                        y =240
                        ind.append([x,y])
                new_line = Line (ind[0][0], ind[0][1], ind[1][0], ind[1][1], theta)
                print(ind[0][0], ind[0][1], ind[1][0], ind[1][1], rho, theta)
                resultant_lines.append (new_line)
        except Exception: pass

        return resultant_lines

    def draw_line (self, line):
        (x1, y1, x2, y2) = line.line()
        cv2.line (self.img, (x1, y1), (x2, y2), (0, 255, 255), thickness = 1)

    def draw_rectangle (self, rect, color = (255, 0, 0) ):
        (x, y, w, h) = rect
        cv2.rectangle (self.img, (x, y), (x+w, y+h), color, 2)

class Sensor:
    
    def __init__ (self, filename_):
        self.filename = filename_
        self.img = cv2.imread (self.filename)
   
    def snapshot (self):
        return Image (self.img.copy ())

class WebCameraSensor(Sensor):
    camera = None
    @staticmethod
    def _cameraInit():
        WebCameraSensor.camera = cv2.VideoCapture(0)
    def __init__(self):
        if (WebCameraSensor.camera == None):
            self._cameraInit()
        self.img = None
    def snapshot(self):
        if (WebCameraSensor.camera.isOpened()):
            ret, self.img = WebCameraSensor.camera.read()
        else:
            print("Camera Closed")
        return Image(self.img.copy())

class KondoCameraSensor(Sensor):
    camera = None
    camera_matrix = None
    dist_matrix = None
    resolution = {"height": 1300, "width": 1600}
    def loadCoefficients(path):
        """ Loads camera matrix and distortion coefficients. """
        # FILE_STORAGE_READ
        print(path)
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        camera_matrix = cv_file.getNode("camera_matrix").mat()
        dist_matrix = cv_file.getNode("dist_coeff").mat()
        cv_file.release()
        print(camera_matrix, dist_matrix)
        return [camera_matrix, dist_matrix]
    @staticmethod
    def _cameraInit(path):
        KondoCameraSensor.camera = arducam.mipi_camera()
        KondoCameraSensor.camera.init_camera()
        KondoCameraSensor.camera.set_mode(6)
        KondoCameraSensor.camera.set_resolution(KondoCameraSensor.resolution["width"],
                KondoCameraSensor.resolution["height"])
        if path != None: KondoCameraSensor.camera_matrix, KondoCameraSensor.dist_matrix = KondoCameraSensor.loadCoefficients(path)

    def align_down(self, size, align):
            return (size & ~((align)-1))

    def align_up(self, size, align):
            return self.align_down(size + align - 1, align)
    

    def __init__(self, path_to_config=None):
        if (KondoCameraSensor.camera == None):
            self._cameraInit(path_to_config)

    def snapshot(self):
        frame = KondoCameraSensor.camera.capture(encoding='raw')
        height = int(self.align_up(KondoCameraSensor.resolution["height"], 16))                                                         
        width = int(self.align_up(KondoCameraSensor.resolution["width"], 32))                                                          
        image = frame.as_array.reshape(int(height), width) # * 1.5                               
        image = cv2.cvtColor(image, cv2.COLOR_BayerRG2BGR) # BG
        (h, w, d) = image.shape
        center = (w // 2, h // 2)
        M = cv2.getRotationMatrix2D(center, 180, 1.0)
        image = cv2.warpAffine(image, M, (w, h))
        return Image(image)
    def undistored_snapshot(self):
        frame = KondoCameraSensor.camera.capture(encoding='raw')
        height = int(self.align_up(KondoCameraSensor.resolution["height"], 16))                                                         
        width = int(self.align_up(KondoCameraSensor.resolution["width"], 32))                                                          
        image = frame.as_array.reshape(int(height), width) # * 1.5                               
        image = cv2.cvtColor(image, cv2.COLOR_BayerRG2BGR) # BG
        (h, w, d) = image.shape
        center = (w // 2, h // 2)
        M = cv2.getRotationMatrix2D(center, 180, 1.0)
        image = cv2.warpAffine(image, M, (w, h))
        image = cv2.undistort(
            image, KondoCameraSensor.camera_matrix, KondoCameraSensor.dist_matrix, None, None
            )
        return Image(image)
    def __del__(self):
        KondoCameraSensor.camera.close_camera()

def main ():
#    sensor = Sensor ("rgb_basket.jpg")
    sensor = KondoCameraSensor()
    while (True):
        #print ("a")
        img = sensor.snapshot ()

        #blobs = img.find_blobs ((40, 80, -28, 72, -28, 72), 200, 20, True, 10)
        blobs = img.find_blobs ([(35, 50, 15, 75, 50, 135)], 200, 20, True, 10)

        for blob in blobs:
            #print ("a")
            img.draw_rectangle (blob.rect ())

        #lines = img.find_lines ()

        #for line in lines:
        #    img.draw_line (line.line ())

        cv2.imshow ("objects", img.img)

        time.sleep (0.02)
        
        keyb = cv2.waitKey (1) & 0xFF
        
        if (keyb == ord('q')):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main ()
