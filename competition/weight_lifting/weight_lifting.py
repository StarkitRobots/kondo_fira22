from competition import Competition
import json
import os
import pyb
from threading import Thread
import numpy as np
import time
class WeightLifting(Competition):
    def __init__(self, button):
        super().__init__(button)
        self.glob.camera_ON = True
        self.cam_proc = Thread(target=self.process_vision)
        self.center_of_barbell = []
        self.detect, self.mediana_of_coords, self.distance = False, (None, None), None
        # st else

    def process_vision(self):
        print("receiving a picture")
        img = self.sensor.snapshot()
        if img is not None:
            print("begining of reload")
            # if name == 'barbell':
            coords = img.find_blobs([self.glob.TH['weight lifting']['thbarbell']], pixels_threshold=self.glob.TH['weight lifting']['pixel'], area_threshold=self.glob.TH['weight lifting']['area'], merge=True)
            print(f"number of found objects: {len(coords)}")
            # elif name == 'barbell':
            self.center_of_barbell = []
            for coord in coords:
                c_barbell_x = coord.cx()
                c_barbell_y = coord.cy()
                center_of_barbell0 = (c_barbell_x, c_barbell_y)   #(coord.cx(), coord.cy())  #pixel's (x, y) coordinates of object's center
                self.center_of_barbell.append(center_of_barbell0)
                print(f"pixel coordinates: {center_of_barbell0}")
            if len(self.center_of_barbell) != 0:
                print("barbell is seen")
                print(f"There are {len(self.center_of_barbell)} barbells. Their pixel coordinates are {self.center_of_barbell}")
        else:
            self.center_of_barbell = (None, None)
            print("img is None")

    def radians_search(self, frequency):
        res = list(zip(np.zeros(frequency), np.linspace(-np.pi/2, np.pi/2, frequency))) + list(zip(np.zeros(frequency) - np.pi/4, np.linspace(np.pi/2, -np.pi/2, frequency)))
        return res + [[0, 0]]

    def get_distance(self, pixels):
        coords = self.motion.self_coords_from_pixels(pixels[0], pixels[1], "barbell")
        print(f"Final coordinates of barbell: {coords}")
        return coords

    def finding(self):
        flag, mediana_of_coords, distance = False, (None, None), None
        print("Start finding barbell")
        radians = self.radians_search(5)
        l = []
        while len(l) < 2:
            for elem in radians:
                self.motion.move_head(elem[1]*1000, elem[0]*1000) # NEED TO MOVE HEAD!!!
                time.sleep(3)
                self.process_vision()
                if self.center_of_barbell != (None, None):
                    coords = self.get_distance(self.center_of_barbell)
                    print(coords)
                    l.append(coords)
            # choose the mediana of all balls in coordinates of robor and look at dispersy, check massive of all finded balls
                else:
                    print("self.center_of_barbell == (None, None)")
        mediana_of_coords = tuple(np.median(np.array(l), axis = 0))
        print(f"MEDIANA: {mediana_of_coords}")
        if mediana_of_coords != (None, None):
            flag = True
            distance = np.sqrt(mediana_of_coords[0] ** 2 + mediana_of_coords[1] ** 2)
        return flag, mediana_of_coords, distance


    def run(self):
        print("running")
        while self.detect == False:     # пока штанга не найдена, искать
            self.detect, self.mediana_of_coords, self.distance = self.finding()
        print(f"Barbell detection is {self.detect}. mediana = {self.mediana_of_coords}. distance = {self.distance}")

if __name__ == '__main__':
    default_button = 1
    weight_lifter = WeightLifting(default_button)
    weight_lifter.run()
    