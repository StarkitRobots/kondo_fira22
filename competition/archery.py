from turtle import position
from sprint import Competition




import sys
import os
import math
import json
import time
import cv2

current_work_directory = os.getcwd()
current_work_directory = current_work_directory.replace('\\', '/')
import pyb
current_work_directory += '/'
SIMULATION = 2                                         # 0 - Simulation without physics, 1 - Simulation synchronous with physics, 2 - live on open
sys.path.append( current_work_directory + 'Soccer/')
sys.path.append( current_work_directory + 'Soccer/Motion/')
SIMULATION=2
from class_Motion import Glob
from reload import KondoCameraSensor
from class_Motion import Motion1 as Motion




class Sprint(Competition):
    def __init__(self, path_to_camera_config):   
        self.glob = Glob(SIMULATION, current_work_directory)
        self.motion = Motion(self.glob)
        self.motion.activation()
        self.motion.falling_Flag = 0
        self.number_of_cycles = 3000000 #30
        self.motion.simThreadCycleInMs = 20 # 20
        self.motion.amplitude = 32 #32
        self.motion.fr1 = 4 # 4
        self.motion.fr2 = 10 # 10
        ##self.motion.initPoses = self.motion.fr2 
        self.motion.gaitHeight = 190 # 190
        self.motion.stepHeight = 40  # 20
        self.stepLength = 70 #70 
        self.sensor = KondoCameraSensor(path_to_camera_config)
        self.aruco_init()
    def aruco_init(self):
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
    def aruco_position(self, img):
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, self.arucoDict,
                        parameters=self.arucoParams)
        print(corners)
        print(self.sensor.camera_matrix, self.sensor.dist_matrix)
        if corners != []:
            tvec, rvec = cv2.aruco.estimatePoseSingleMarkers(corners[0], 0.05, self.sensor.camera_matrix, self.sensor.dist_matrix)
        else : return [[[0,0,0]]],[[[0,0,0]]]
        print("rvec : ", rvec)
        print("tvec : ", tvec)    
        return tvec, rvec

    def run_forward_1(self):
        for cycle in range(self.number_of_cycles):
            img = self.sensor.snapshot().img
            rvec, tvec = self.aruco_position(img)
            rotation = 0 
            if cycle ==0 : stepLength1 = self.stepLength/4
            if cycle ==1 : stepLength1 = self.stepLength/2
            if cycle ==2 : stepLength1 = self.stepLength/4 * 3
            self.motion.walk_Cycle(stepLength1,0,max(rvec[0][0][1],0.1) if rvec[0][0][1] > 0 else max(rvec[0][0][1],0.1),cycle, self.number_of_cycles)






#!/usr/bin/env python3
import math
import time
import cv2
import numpy as npfrom 
from scipy import optimize
import model

# class ExampleFSM:
#     def __init__(self):
#         self.ball = (None, None)
#         self.last_seen_ball = 0
#         self.camera_pan = 0
#         self.camera_tilt = 0

#     def update_ball(self, msg):
#         self.ball = (msg.x, msg.y)
#         self.last_seen_ball = time.time()
#         rospy.loginfo(f"Got the ball: {self.ball[0]}, {self.ball[1]}")
#         self.ball_self = self.get_ball_distance()
#         rospy.loginfo(f"Got the ball in self coords: {self.ball_self[0]}, {self.ball_self[1]}")


#     def get_ball_distance(self):
#         rospy.wait_for_service('model_service')
#         try:
#             get_coords = rospy.ServiceProxy('model_service', ModelService)
#             print(self.ball[0], self.ball[1], self.camera_pan, self.camera_tilt)
#             response = get_coords(int(self.ball[0]), int(self.ball[1]), self.camera_pan, self.camera_tilt)
#             return (response.x, response.y)
#         except rospy.ServiceException as e:
#             print("Service call failed: %s"%e)

    
#     def tick(self):
#         # rospy.spinOnce()
#         if (time.time() - self.last_seen_ball <= 1.0):
#             rospy.loginfo("Go to the ball")
#         else: 
#             rospy.loginfo("Searching for the ball")


def detect_circle_params(x_coords, y_coords):    
    def calc_R(xc, yc):
        return np.sqrt((x_coords-xc)**2 + (y_coords-yc)**2)

    def f_2b(c):
        Ri = calc_R(*c)
        return Ri - Ri.mean()

    def Df_2b(c):
        xc, yc     = c
        df2b_dc    = np.empty((len(c), len(x_coords)))

        Ri = calc_R(xc, yc)
        df2b_dc[0] = (xc - x_coords)/Ri                   # dR/dxc
        df2b_dc[1] = (yc - y_coords)/Ri                   # dR/dyc
        df2b_dc    = df2b_dc - df2b_dc.mean(axis=1)[:, np.newaxis]

        return df2b_dc

    x_m = np.mean(x_coords)
    y_m = np.mean(y_coords)
    center_estimate = x_m, y_m
    center_2b, ier = optimize.leastsq(f_2b, center_estimate, Dfun=Df_2b, col_deriv=True)

    xc_2b, yc_2b = center_2b
    Ri_2b        = calc_R(*center_2b)
    R_2b         = Ri_2b.mean()
    residu_2b    = sum((Ri_2b - R_2b)**2)

    return xc_2b, yc_2b, R_2b, residu_2b

def calculate_period(positions): #measurments):
    def dotproduct(v1, v2):
        return sum((a*b) for a, b in zip(v1, v2))

    def length(v):
        return math.sqrt(dotproduct(v, v))

    def angle_b2v(v1, v2):
        cos = dotproduct(v1, v2) / (length(v1) * length(v2))
        angle = 0 if cos > 1 else math.pi if cos < -1 else math.acos(cos)
        return angle if angle > 0 else angle + 2 * math.pi

    def compute_angle(x1, y1, x2, y2, x_circle, y_circle):
        x1_d = x1 - x_circle
        y1_d = y1 - y_circle
        x2_d = x2 - x_circle
        y2_d = y2 - y_circle
        return angle_b2v((x1_d, y1_d), (x2_d, y2_d))

    #sort by time
    # measurments = sorted(measurments, key=lambda x: x[2])
    # x_coords, y_coords, time_stamps = zip(*measurements)

    # time_stamps = timestamps
    x_coords, y_coords = zip (*positions)

    measurements = list (zip (x_coords, y_coords, timestamps))
    # print (measurements)

    circle_x, circle_y, circle_r, circle_error = detect_circle_params(x_coords, y_coords)
    # circle_x, circle_y, circle_r = (np.max(x_coords) + np.min(x_coords))/2, \
    #                               (np.max(y_coords) + np.min(y_coords))/2, \
    #                               ((np.max(x_coords) - np.min(x_coords) + \
    #                                 np.max(y_coords) - np.min(y_coords))/4)

    angle = 0
    for st1, st2 in zip(measurements[:-1], measurements[1:]):
        angle += compute_angle(st2[0], st2[1], st1[0], st1[1], circle_x, circle_y)

    # all_time = measurements[-1][2] - measurements[0][2]
    # period = all_time / angle * 2 * math.pi
    return  circle_x, circle_y, circle_r, circle_error

def find_target_center(frame):
    # cv2.imshow("real", frame)
    avarage_x = 0
    avarage_y = 0
    
#Switch to HSV 
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
#Detect blue circle
    mask_blue = cv2.inRange(hsv, (79,76,115), (125,199,212))
    
    blured_blue = cv2.medianBlur(mask_blue*gray, 5)
    
    kernel_blue = np.ones((5,5), np.uint8)
    img_erosion_blue = cv2.erode(blured_blue, kernel_blue, iterations=1)
    img_dilation_blue = cv2.dilate(img_erosion_blue, kernel_blue, iterations=1)
    
    canny_blue = cv2.Canny(img_dilation_blue, 350, 360)
    
    # cv2.imshow("blue", canny_blue)
    
    circle_blue = cv2.HoughCircles(canny_blue, cv2.HOUGH_GRADIENT, 1.4, 100)
    
#Detect yellow circle
    mask_yellow = cv2.inRange(hsv, (8,60,140), (32,162,229))
    
    blured_yellow = cv2.medianBlur(mask_yellow*gray, 5)
    
    kernel_yellow = np.ones((5,5), np.uint8)
    img_erosion_yellow = cv2.erode(blured_yellow, kernel_yellow, iterations=1)
    img_dilation_yellow = cv2.dilate(img_erosion_yellow, kernel_yellow, iterations=1)
    
    canny_yellow = cv2.Canny(img_dilation_yellow, 180, 190)
    
    # cv2.imshow("yellow", canny_yellow)
    
    circle_yellow = cv2.HoughCircles(canny_yellow, cv2.HOUGH_GRADIENT, 1.4, 100)
    
#Detect red circle
    mask_red = cv2.inRange(hsv, (0,52,93), (18,229,228))
    
    blured_red = cv2.medianBlur(mask_red*gray, 7)
    
    kernel_red = np.ones((7,7), np.uint8)
    img_erosion_red = cv2.erode(blured_red, kernel_red, iterations=1)
    img_dilation_red = cv2.dilate(img_erosion_red, kernel_red, iterations=1)
    
    canny_red = cv2.Canny(img_dilation_red, 350, 360) 
    
    # cv2.imshow("red", canny_red)
    
    circle_red = cv2.HoughCircles(canny_red, cv2.HOUGH_GRADIENT, 2.2, 100)
    
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
        # print(avarage_x, avarage_y)
        # # draw a rectangle
        # # corresponding to the center of the circles
        # cv2.rectangle(output, (avarage_x - 5, avarage_y - 5), (avarage_x + 5, avarage_y + 5), (0, 128, 255), -1)
        # # show the output image
        # cv2.imshow("output", np.hstack([frame, output]))
        # cv2.waitKey(10)
    return avarage_x , avarage_y

class ArcheryFSM:
    def __init__(self):












        self.cam_frame = None
        self.traj_coords = []
        self.timestamps = []

        self.pelvis_rot = 0
        self.pelvis_rot_mistake = 0.1
        self.pelvis_rot_const = 1.4
        self.pointed_to_target = False

        self.number_of_frames = 100

        self.period = 0
        self.circle_x = 0
        self.circle_y = 0
        self.circle_r = 0
        self.circle_error = 0

        self.l_ind = 0

        self.time_delay = 1.1
        self.time_accuracy = 0.1

    def update_camera_frame(self, msg):
        # self.cam_frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        self.cam_frame = self.motion.sensor.snapshot().img
    def count_pelvis_rotation(self):
            # get_coords = rospy.ServiceProxy("model_service", ModelService)
            # rospy.loginfo(self.circle_x, self.circle_y, 0, 0)
            print(int(self.circle_x))
            response = self.motion.self_coords_from_pixels(int(self.circle_x), int(self.circle_y), "basket")#, 0, 0, 0.555)# get_coords(int(self.circle_x), int(self.circle_y), 0, 0, 0.555)
            return (np.arctan(response.y / response.x))

    def move_pelvis(self):
        self.servos_client(0,2, -self.pelvis_rot*self.pelvis_rot_const)

    def release_the_bowstring(self):
        self.servos_client(13,2, -1)


    def servos_client (self,id,sio position):
        self.motion.set_servo_pos(id,sio,position)


    def motion_client(self, motion_name):
        self.motion.play_Motion_Slot(motion_name)
 
    def predict_time(self):
        def get_norm_vector(point):
            traj_center_coords = [self.circle_x, self.circle_y]
            return np.subtract (point, traj_center_coords)

        def dotproduct(v1, v2):
            return sum((a*b) for a, b in zip(v1, v2))
 
        def length(v):
            return math.sqrt(dotproduct(v, v))
 
        def angle_b2v(v1, v2):
            cos = dotproduct(v1, v2) / (length(v1) * length(v2))
            angle = 0 if cos > 1 else math.pi if cos < -1 else math.acos(cos)
            return angle if angle > 0 else angle + 2 * math.pi

        position = [self.circle_x, self.circle_y + self.circle_r]
        current_vec = get_norm_vector(self.traj_coords [-1])
        point_norm_vec = get_norm_vector(position)
        angle_diff = angle_b2v(current_vec, point_norm_vec)
        angle_diff = angle_diff if angle_diff > 0 else angle_diff + 2 * math.pi
        if(self.traj_coords[-1][0] > self.circle_x):
            angle_diff = 2 * math.pi - angle_diff

        angular_speed = 2 * math.pi / self.period

        predicted_time = angle_diff / angular_speed

        #print (current_vec, angle_diff, angular_speed)

        return predicted_time   

    def tick(self):
        self.update_camera_frame()
        if self.cam_frame is not None:
            if(len(self.traj_coords) > self.number_of_frames) and (not self.pointed_to_target):
                tail_len = self.number_of_frames
                frame = self.cam_frame.copy()
                # output = frame.copy()
                trajectory_x, trajectory_y = find_target_center(frame)
                self.traj_coords.append([trajectory_x, trajectory_y])
                self.timestamps.append(time.time())
                
                if (len(self.traj_coords) > tail_len):
                    l_ind = len(self.traj_coords) - tail_len
                
                # print(len(self.traj_coords), self.l_ind)
                
                self.circle_x, self.circle_y, self.circle_r, self.circle_error = calculate_period(self.traj_coords[self.l_ind : ])
                
                self.pelvis_rot += self.count_pelvis_rotation()
                print(self.pelvis_rot)

                self.pointed_to_target = True

                self.move_pelvis()
                self.traj_coords.clear()


                # Visual output
                # print(self.circle_x, self.circle_y)
                # if circle_x is not None:
                #     if (abs(circle_x) < 5000 and abs(circle_y) < 5000):
                #         cv2.circle(output, (int(circle_x), int(circle_y)), int(circle_r), (0, 255, 0), 4)
                #         cv2.rectangle(output, (int(circle_x) - 5, int(circle_y) - 5), (int(circle_x) + 5, int(circle_y) + 5), (0, 128, 255), -1)
                #         # font
                #         font = cv2.FONT_HERSHEY_SIMPLEX

                #         # org
                #         org = (50, 50)

                #         # fontScale
                #         fontScale = 1

                #         # Blue color in BGR
                #         color = (255, 0, 0)

                #         # Line thickness of 2 px
                #         thickness = 2

                #         # Using cv2.putText() method
                #         output = cv2.putText(output, str(period), org, font, 
                #                     fontScale, color, thickness, cv2.LINE_AA)
                #         cv2.imshow("output", np.hstack([frame, output]))
                #         cv2.waitKey(10)

                #Close on q
                # key = cv2.waitKey(50) & 0xFF
                # if (key == ord('q')):
                #     break
            elif(len(self.traj_coords) > self.number_of_frames) and (self.pointed_to_target):
                tail_len = self.number_of_frames
                self.cam_frame = self.update_camera_frame()
                frame = self.cam_frame.copy()
                trajectory_x, trajectory_y = find_target_center(frame)
                self.traj_coords.append([trajectory_x, trajectory_y])
                # self.timestamps.append(time.time())
                
                if (len(self.traj_coords) > tail_len):
                    l_ind = len(self.traj_coords) - tail_len
                
                self.circle_x, self.circle_y, self.circle_r, self.circle_error = calculate_period(self.traj_coords[self.l_ind : ], self.timestamps)

                # pred_time = self.predict_time()
                # print(pred_time)

                # if((pred_time - self.time_delay) < self.time_accuracy):
                #     return False
            else:
                self.cam_frame = self.update_camera_frame()
                frame = self.cam_frame
                trajectory_x, trajectory_y = find_target_center(frame)
                self.traj_coords.append([trajectory_x, trajectory_y])
                # self.timestamps.append(time.time())
        # cam.release()
        # cv2.destroyAllWindows()
        # cv2.waitKey(10)
        return True


if __name__ == "__main__":
    # rospy.init_node("archery")
    archery = Archery()
    image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, archery.update_camera_frame)

    archery.motion_client("archery_ready")  #ACTION 1
    input()
    archery.motion_client("archery_setup")  #ACTION 2
    time.sleep(4)
    archery.motion_client("archery_pull")   #ACTION 3
    to_continue = True
    while to_continue:
        time.sleep(0.05)
        to_continue = archery.tick()
    print('shoot')
    archery.release_the_bowstring()