#!/usr/bin/env python3
from competition import Competition
from os import system
# from turtle import position
import sys
import os
import math
import json
import time
import cv2
from threading import Thread
import numpy as np 
from scipy import optimize

# def detect_circle_params(x_coords, y_coords):    
#     def calc_R(xc, yc):
#         return np.sqrt((x_coords-xc)**2 + (y_coords-yc)**2)

#     def f_2b(c):
#         Ri = calc_R(*c)
#         return Ri - Ri.mean()

#     def Df_2b(c):
#         xc, yc     = c
#         df2b_dc    = np.empty((len(c), len(x_coords)))

#         Ri = calc_R(xc, yc)
#         df2b_dc[0] = (xc - x_coords)/Ri                   # dR/dxc
#         df2b_dc[1] = (yc - y_coords)/Ri                   # dR/dyc
#         df2b_dc    = df2b_dc - df2b_dc.mean(axis=1)[:, np.newaxis]

#         return df2b_dc

#     x_m = np.mean(x_coords)
#     y_m = np.mean(y_coords)
#     center_estimate = x_m, y_m
#     center_2b, ier = optimize.leastsq(f_2b, center_estimate, Dfun=Df_2b, col_deriv=True)

#     xc_2b, yc_2b = center_2b
#     Ri_2b        = calc_R(*center_2b)
#     R_2b         = Ri_2b.mean()
#     residu_2b    = sum((Ri_2b - R_2b)**2)

#     return xc_2b, yc_2b, R_2b, residu_2b

# def calculate_period(positions): #measurments):
#     def dotproduct(v1, v2):
#         return sum((a*b) for a, b in zip(v1, v2))

#     def length(v):
#         return math.sqrt(dotproduct(v, v))

#     def angle_b2v(v1, v2):
#         cos = dotproduct(v1, v2) / (length(v1) * length(v2))
#         angle = 0 if cos > 1 else math.pi if cos < -1 else math.acos(cos)
#         return angle if angle > 0 else angle + 2 * math.pi

#     def compute_angle(x1, y1, x2, y2, x_circle, y_circle):
#         x1_d = x1 - x_circle
#         y1_d = y1 - y_circle
#         x2_d = x2 - x_circle
#         y2_d = y2 - y_circle
#         return angle_b2v((x1_d, y1_d), (x2_d, y2_d))

#     #sort by time
#     # measurments = sorted(measurments, key=lambda x: x[2])
#     # x_coords, y_coords, time_stamps = zip(*measurements)

#     # time_stamps = timestamps
#     x_coords, y_coords = zip (*positions)

#     measurements = list (zip (x_coords, y_coords))
#     # print (measurements)

#     circle_x, circle_y, circle_r, circle_error = detect_circle_params(x_coords, y_coords)
#     # circle_x, circle_y, circle_r = (np.max(x_coords) + np.min(x_coords))/2, \
#     #                               (np.max(y_coords) + np.min(y_coords))/2, \
#     #                               ((np.max(x_coords) - np.min(x_coords) + \
#     #                                 np.max(y_coords) - np.min(y_coords))/4)

#     angle = 0
#     for st1, st2 in zip(measurements[:-1], measurements[1:]):
#         angle += compute_angle(st2[0], st2[1], st1[0], st1[1], circle_x, circle_y)

#     # all_time = measurements[-1][2] - measurements[0][2]
#     # period = all_time / angle * 2 * math.pi
#     return  circle_x, circle_y, circle_r, circle_error

# class Archery(Competition):
#     def __init__(self):
#         self.traj_coords = []
#         self.timestamps = []
#         self.target_current_x = self.target_current_y = None

#         self.target_desired_x = 800
#         self.pelvis_rot_mistake_in_pixels = 0.08
#         self.pointed_to_target = False

#         self.targeting_kp = 0.0013

#         self.number_of_frames = 1

#         self.period = 0
#         self.circle_x = 0
#         self.circle_y = 0
#         self.circle_r = 0
#         self.circle_error = 0

#         self.l_ind = 0

#         self.time_delay = 1.1
#         self.time_accuracy = 0.1
        
#         # self.sensor = KondoCameraSensor("/home/pi/kondo_fira22/Camera_calibration/mtx.yaml")

#         self.cam_proc = Thread(target=self.process_vision)
#         self.stopFlag = False
    
#     # def count_pelvis_rotation(self):
#     #         # get_coords = rospy.ServiceProxy("model_service", ModelService)
#     #         # rospy.loginfo(self.circle_x, self.circle_y, 0, 0)
#     #         print(int(self.circle_x))
#     #         response = self.motion.self_coords_from_pixels(int(self.circle_x), int(self.circle_y), "basket")#, 0, 0, 0.555)# get_coords(int(self.circle_x), int(self.circle_y), 0, 0, 0.555)
#     #         return (np.arctan(response[1] / response[0]))

#     def move_pelvis(self, angle):
#         self.motion.set_servo_pos(0,2, angle)

#     def release_the_bowstring(self):
#         self.motion.set_servo_pos(13,1, -1)


#     def motion_client(self, motion_name):
#         self.motion.play_Motion_Slot(motion_name)
 
#     def predict_time(self):
#         def get_norm_vector(point):
#             traj_center_coords = [self.circle_x, self.circle_y]
#             return np.subtract (point, traj_center_coords)

#         def dotproduct(v1, v2):
#             return sum((a*b) for a, b in zip(v1, v2))
 
#         def length(v):
#             return math.sqrt(dotproduct(v, v))
 
#         def angle_b2v(v1, v2):
#             cos = dotproduct(v1, v2) / (length(v1) * length(v2))
#             angle = 0 if cos > 1 else math.pi if cos < -1 else math.acos(cos)
#             return angle if angle > 0 else angle + 2 * math.pi

#         position = [self.circle_x, self.circle_y + self.circle_r]
#         current_vec = get_norm_vector(self.traj_coords [-1])
#         point_norm_vec = get_norm_vector(position)
#         angle_diff = angle_b2v(current_vec, point_norm_vec)
#         angle_diff = angle_diff if angle_diff > 0 else angle_diff + 2 * math.pi
#         if(self.traj_coords[-1][0] > self.circle_x):
#             angle_diff = 2 * math.pi - angle_diff

#         angular_speed = 2 * math.pi / self.period

#         predicted_time = angle_diff / angular_speed

#         print (current_vec, angle_diff, angular_speed)

#         return predicted_time   

#     def process_vision(self):
#         print(self.stopFlag)
#         print("receiving a picture")
#         img = self.sensor.snapshot()
#         if img is not None:
#             print("begining of reload")
#             self.target_current_x, self.target_current_y = img.find_target_center(self.glob.TH['archery']['thblue'], self.glob.TH['archery']['thyellow'], self.glob.TH['archery']['thred'])
#             if (self.target_current_x, self.target_current_y) != (None, None):       print(f"Coordinates of center of target are {(self.target_current_x, self.target_current_y)}")
#         else:
#              print("XYETA")
#             #  self.target_current_x = target_current_y = None

#     def tick(self):
#         print(f"************* {self.target_current_x} **************************** {self.target_current_y}")
#         if self.target_current_x is not None and self.target_current_x > 0:
#            angle_to_turn = self.targeting_kp * (self.target_current_x - self.target_desired_x)
#            print(angle_to_turn)
#            print(f"penis is {self.pelvis_rot_mistake_in_pixels}")
#            if abs(angle_to_turn) >= self.pelvis_rot_mistake_in_pixels:
#                print("starting turning")
#                self.motion.set_servo_pos(0, 2, angle_to_turn)
#            else:
#                self.pointed_to_target = True
#         return self.pointed_to_target

# if __name__ == "__main__":
#     # rospy.init_node("archery")
#     archery = Archery()
#     # archery.release_the_bowstring()   # archery.motion_client("archery_ready")  #ACTION 1
#     # time.sleep(2)
#     # input()
    
#     # print ("archery ready")
#     archery.motion_client("archery_setup")  #ACTION 2
#     time.sleep(12)
    
#     pointed = False
#     x = time.time()
#     while not pointed:
#         archery.process_vision()
#         #time.sleep(archery.time_accuracy)
#         pointed = archery.tick()
#         if time.time() - x > 25:
#             pointed = True
#         if pointed:
#             print('shoot')

#     print("****** HE IS READY TO SHOOT ******")
#     system("mpg123 -q /home/pi/Desktop/bomb-has-been-planted-sound-effect-cs-go.mp3")
#     #system("mpg123 -q /home/pi/Desktop/movie_1.mp3")
#     for i in range (0, 8):
#         print((8 - i))
#         time.sleep(1)
#     archery.motion_client("archery_pull")   #ACTION 3
#     print("ZDOROVA")
#     #archery.release_the_bowstring()


class Archery(Competition):
    def __init__(self, button):
        super().__init__(button)
        self.working = 1
        self.traj_coords = []
        self.timestamps = []
        self.target_current_x = self.target_current_y = None

        self.target_desired_x = 800
        self.pelvis_rot_mistake_in_pixels = 0.08
        self.pointed_to_target = False

        self.targeting_kp = 0.0013

        self.number_of_frames = 1

        self.period = 0
        self.circle_x = 0
        self.circle_y = 0
        self.circle_r = 0
        self.circle_error = 0

        self.l_ind = 0

        self.time_delay = 1.1
        self.time_accuracy = 0.1

        self.cam_proc = Thread(target=self.process_vision)
        self.stopFlag = False

    def move_pelvis(self, angle):
        self.motion.set_servo_pos(0,2, angle)

    def release_the_bowstring(self):
        self.motion.set_servo_pos(13,1, -1)


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

        print (current_vec, angle_diff, angular_speed)

        return predicted_time   

    def process_vision(self):
        print(self.stopFlag)
        print("receiving a picture")
        img = self.sensor.snapshot()
        if img is not None:
            print("begining of reload")
            self.target_current_x, self.target_current_y = img.find_target_center(self.glob.TH['archery']['thblue'], self.glob.TH['archery']['thyellow'], self.glob.TH['archery']['thred'])
            if (self.target_current_x, self.target_current_y) != (None, None):       print(f"Coordinates of center of target are {(self.target_current_x, self.target_current_y)}")
        else:
             print("XYETA")
            #  self.target_current_x = target_current_y = None

    def tick(self):
        print(f"************* {self.target_current_x} **************************** {self.target_current_y}")
        if self.target_current_x is not None and self.target_current_x > 0:
           angle_to_turn = self.targeting_kp * (self.target_current_x - self.target_desired_x)
           print(angle_to_turn)
           print(f"penis is {self.pelvis_rot_mistake_in_pixels}")
           if abs(angle_to_turn) >= self.pelvis_rot_mistake_in_pixels:
               print("starting turning")
               self.motion.set_servo_pos(0, 2, angle_to_turn)
           else:
               self.pointed_to_target = True
        return self.pointed_to_target
    
    def run(self):
        self.motion_client("archery_setup")  #ACTION 2
        time.sleep(12)
        
        pointed = False
        x = time.time()
        while not pointed:
            self.process_vision()
            #time.sleep(archery.time_accuracy)
            pointed = self.tick()
            if time.time() - x > 25:
                pointed = True
            if pointed:
                print('shoot')

        print("****** HE IS READY TO SHOOT ******")
        system("mpg123 -q /home/pi/Desktop/bomb-has-been-planted-sound-effect-cs-go.mp3")
        #system("mpg123 -q /home/pi/Desktop/movie_1.mp3")
        for i in range (0, 8):
            print((8 - i))
            time.sleep(1)
        self.motion_client("archery_pull")   #ACTION 3
        print("ZDOROVA")

if __name__ == '__main__':
    default_button = 1
    archer = Archery(default_button)
    archer.run()