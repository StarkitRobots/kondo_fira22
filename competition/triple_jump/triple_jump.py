from competition import Competition
import numpy as np
import time

class TripleJump(Competition):
    def __init__(self, button):
        super().__init__(button)
        self.glob.camera_ON = True
        self.center_of_bottom = (None, None)
        self.detect, self.mediana_of_coords, self.distance = False, (None, None), None
        # st else
    
    def radians_search(self, frequency):
        res = list(zip(np.zeros(frequency), np.linspace(-np.pi/2, np.pi/2, frequency))) + list(zip(np.zeros(frequency) - np.pi/4, np.linspace(np.pi/2, -np.pi/2, frequency)))
        return res + [[0, 0]]

    def get_distance(self, pixels):
        coords = self.motion.self_coords_from_pixels(pixels[0], pixels[1], "stripe")
        print(f"Final coordinates of stripe: {coords}")
        return coords

    def process_vision(self):
        print("receiving a picture")
        img = self.sensor.snapshot()
        center_of_bottom = []
        if img is not None:
            print("begining of reload")
            # if name == 'stripe':
            coords = img.find_blobs([self.glob.TH['yellow stripe']['th']], pixels_threshold=self.glob.TH['yellow stripe']['pixel'], area_threshold=self.glob.TH['yellow stripe']['area'], merge=True)
            print(f"number of found objects: {len(coords)}")
            # elif name == 'stripe':
            for coord in coords:
                c_bottom_x = coord.cx()
                c_bottom_y = coord.y() + coord.h()
                center_of_bottom0 = (c_bottom_x, c_bottom_y)   #(coord.cx(), coord.cy())  #pixel's (x, y) coordinates of object's center
                center_of_bottom.append(center_of_bottom0)
                print(f"pixel coordinates: {center_of_bottom0}")
            if len(center_of_bottom) != 0:
                print("bottom is seen!")
                self.center_of_bottom = center_of_bottom[0]
                print(f"There are {len(self.center_of_bottom)} bottoms. Their pixel coordinates are {self.center_of_bottom}")
        else:
            self.center_of_bottom = (None, None)
            print("img is None")

    def finding(self):
        flag, mediana_of_coords, distance = False, (None, None), None
        print("Start finding stripe")
        radians = self.radians_search(5)
        l = []
        while len(l) < 2:
            for elem in radians:
                self.motion.move_head(elem[1]*1000, elem[0]*1000) # NEED TO MOVE HEAD!!!
                time.sleep(3)
                self.process_vision()
                if self.center_of_bottom != (None, None):
                    coords = self.get_distance(self.center_of_bottom)
                    l.append(coords)
                else:
                    print("self.center_of_bottom == (None, None)")
        mediana_of_coords = tuple(np.median(np.array(l), axis = 0))
        print(f"MEDIANA: {mediana_of_coords}")
        if mediana_of_coords != (None, None):
            flag = True
            distance = np.sqrt(mediana_of_coords[0] ** 2 + mediana_of_coords[1] ** 2)
        return flag, mediana_of_coords, distance

    def run(self):
        print("running")
        while self.detect == False:     # пока полоска не найдена, искать
            self.detect, self.mediana_of_coords, self.distance = self.finding()
        print(f"Stripe detection is {self.detect}. mediana = {self.mediana_of_coords}. distance = {self.distance}")
if __name__ == '__main__':
    default_button = 1
    triple_jumper = TripleJump(default_button)
    triple_jumper.run()