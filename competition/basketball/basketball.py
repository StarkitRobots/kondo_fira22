import numpy as np
import time
import math

# THIS IS NEED TO BE FOUND OR WRITTEN
# from approach import approach
# from center_point import center_point
# from localisation import localisation

from competition import Competition

class Basketball(Competition):
    def __init__(self, button):
        super().__init__(button)
        # st else

    def get_pixels(self, name):
        center = []
        self.glob.camera_ON = True
        center = self.motion.check_camera(name)
        pixels = center[0]
        return pixels

    def get_distance(self, pixels, name):
        coords = self.motion.self_coords_from_pixels(pixels[0], pixels[1], name)
        print(f"coordinates of object: {coords}")
        return coords

    def radians_search(self, frequency):
        res = list(zip(np.zeros(frequency), np.linspace(-np.pi/2, np.pi/2, frequency))) + list(zip(np.zeros(frequency) - np.pi/4, np.linspace(np.pi/2, -np.pi/2, frequency)))
        return res + [[0, 0]]

    def finding(self, name):
        flag = False
        mediana_of_coords = (None, None)
        distance = 0
        print("Start finding ball")
        radians = self.radians_search(5)
        l = []
        while len(l) < 2:
            for elem in radians:
                self.motion.move_head(elem[1]*1000, elem[0]*1000) # NEED TO MOVE HEAD!!!
                time.sleep(3)
                pixels = self.get_pixels(name)
                if pixels != (None, None):
                    coords = self.get_distance(pixels, name)
                    print(coords)
                    l.append(coords)
            # choose the mediana of all balls in coordinates of robor and look at dispersy, check massive of all finded balls
                else:
                    print("GOVNO")
        mediana_of_coords = tuple(np.median(np.array(l), axis = 0))
        print(f"MEDIANA: {mediana_of_coords}")
        if mediana_of_coords != (None, None):
            flag = True
            distance = np.sqrt(mediana_of_coords[0] ** 2 + mediana_of_coords[1] ** 2)
        return flag, mediana_of_coords, distance

    '''
    def rotate(to_rotate_deg):    #XYETAAAAAAA

        imu_start = self.imu_client().x
        imu_end = imu_start - to_rotate_deg
        imu_end %= 360
        rotation = 0.2
        print(to_rotate_deg)

        while True:
            imu = self.imu_client()
            print("imu: ", imu.x)
            print("imu end: ", imu_end)
            if np.abs(imu.x - imu_end) < 3:
                break
            
            if imu.x - imu_end > 0:
                if math.fabs(imu.x - imu_end) <= 180:
                    self.walk_client(True, 0, 0, rotation)
                else:
                    self.walk_client(True, 0, 0, -rotation)
            if  imu.x - imu_end <= 0:
                print("Negative")
                if math.fabs(imu.x - imu_end) <= 180:
                    self.walk_client(True, 0, 0, -rotation)
                else:
                    self.walk_client(True, 0, 0, rotation)
            
        self.walk_client(False, 0, 0, 0)
    '''

    def turn_to(self, coords):
        print("Start turning to ball/basketcase")
        degree = np.arctan(coords[1] / coords[0]) * 180 / np.pi
        print(str(degree))
        # rotate(degree)      #THIS FUNCTION NEEDS ANOTHER VIEW
        time.sleep(1)

    def go_to(self, percent_distance, coords, distance):
        print("Start going")
        r_x, r_y, r_theta = 1, 2, 0 #robot's coordinates
    #img = Image(cv2.imread("Soccer\\5837.png", cv2.COLOR_BGR2LAB))
    #blob = img.find_blobs([[100, 110, 140, 210, 220, 230], [50, 70, 80, 230, 240, 250], [10, 20, 30, 251, 252, 253]], 30, 30)
    
        point = [100, 100, 100]
        acc = 0.001
        cycle = 2
        number_of_cycles = 30 #30
        self.motion.simThreadCycleInMs = 10
        self.motion.amplitude = 0 #32
        self.motion.fr1 = 8 # 4
        self.motion.fr2 = 12
        ##self.motion.initPoses = self.motion.fr2 
        self.motion.gaitHeight = 160
        self.motion.stepHeight = 40  # 20
        stepLength = 40
        sideLength = 0
        #self.motion.first_Leg_Is_Right_Leg = False
        self.motion.walk_Initial_Pose()
        while math.fabs(r_x - point[0]) >= acc or math.fabs(r_y - point[1]) >= acc or math.fabs(r_theta - point[2]) >= acc:
            point = self.center_point()
            r_x = coords[0]
            r_y = coords[1]
            
            steps = self.approach(r_x, r_y, r_theta, point[0], point[1], point[2])
            try:
                r_x, r_y, r_theta = self.localisation(steps[1])
                print(r_x, r_y, r_theta)
                if self.motion.first_Leg_Is_Right_Leg: invert = -1
                else: invert = 1
                
                if not self.motion.falling_Flag == 0: break
                stepLength1 = stepLength
                if cycle == 0 : stepLength1 = stepLength/3
                if cycle == 1 : stepLength1 = stepLength/3 * 2
                if (r_x - point[0])**2 + (r_x - point[1])**2 <= stepLength: stepLength1 = math.sqrt((r_x - point[0])**2 + (r_x - point[1])**2)
                #rotation = steps[0][2]
                #rotation = self.motion.normalize_rotation(rotation) 
                rotation = 0
                number_of_cycles += 1
                self.motion.walk_Cycle(stepLength1, sideLength, rotation, cycle, number_of_cycles)
                if not self.motion.falling_Flag == 0:
                    if self.motion.falling_Flag == 3: 
                        print('STOP!')
                        return
                    else: 
                        print('FALLING!!!', self.motion.falling_Flag)
                        self.motion.falling_Flag = 0
                        continue
                cycle += 1
                number_of_cycles += 1
                
            except IndexError:
                r_x, r_y, r_theta = point[0], point[1], point[2]
        self.motion.walk_Final_Pose()
        #self.motion.simulateMotion(name = 'Kondo3_TripleJump')
            ## NEED TO GO TO THE BALL!!!

    def thinking_take(self, ball_coordinates, basketcase_coordinates):
        print("Start thinking and taking ball") 
        basketcase_linia = ((basketcase_coordinates[0]) / (basketcase_coordinates[1]))
        angle_thinking = np.pi/2 - np.arctanh(basketcase_linia)
        if ball_coordinates != (None, None):
            distance = (math.cos(angle_thinking)) * np.sqrt(ball_coordinates[0] ** 2 + ball_coordinates[1] ** 2)
        if math.fabs(distance) > 0.16:
            # TAKE THE BALL
            self.motion.play_Soft_Motion_Slot(name = 'basketball_taking_ball')
            time.sleep(1)
            # pox, it won't bring down the holder
            return True
        elif distance < 0:
            # TAKE THE BALL
            self.motion.play_Soft_Motion_Slot(name = 'basketball_taking_ball')
            time.sleep(1)
            # 3 steps right NEED TO DO!!!
            return True
        elif distance >= 0:
            # TAKE THE BALL
            self.motion.play_Soft_Motion_Slot(name = 'basketball_taking_ball')
            time.sleep(1)
            # 3 steps left NEED TO DO!!!
            return True
        else:
            return False 

    def run(self):
        #self.motion.play_Soft_Motion_Slot(name = 'Ball_tennis_throw_v7_2')
        #self.motion.kondo.motionPlay(78) 
        #self.motion.activation()
        #self.motion.falling_Flag = 0
        
        #PROVERKA OF VISION
            
        while True:
            pixels = self.get_pixels('basket')
            if pixels != (None, None):
                coords = self.get_distance(pixels, 'basket')
                print(f"coordinates of basket: {coords}")
            pixels = self.get_pixels('ball')
            if pixels != (None, None):
                coords = self.get_distance(pixels, 'ball')
                print(f"coordinates of ball: {coords}")
        
        #END OF PROVERKA OF VISION
        
        '''

        # THE BEGINING
        self.glob.camera_ON = False
        flag_ball = False
        flag_basket = False
        flag_evade = False

        # self.motion.move_head(1000, -2000)
        # Finding ball and putting it to self.ball_coordinates for future approach.

        while not flag_ball:
            flag_ball, ball_coords, ball_distance = finding('ball')      # (True/False), (x,y), distance
        print(f"result of finding ball: {ball_coords} and {ball_distance}")
        

        #Approaching to the ball on 0.8 of distance
        
        flag_ball = False
        turn_to(ball_coords)    #DOESN'T WORK NOW
        # maybe do finding one more time
        go_to(0.8, ball_coords, ball_distance)  #DOESN'T WORK NOW
        
        # Correct the ball position

        while not flag_ball:
            flag_ball, ball_coords, ball_distance = finding('ball')      # (True/False), (x,y), distance
        print(f"result of finding ball: {ball_coords} and {ball_distance}")

        # Finally approach a ball
        
        flag_ball = False
        turn_to(ball_coords)    #DOESN'T WORK NOW
        # maybe do finding one more time
        go_to(1, ball_coords, ball_distance)  #DOESN'T WORK NOW

        # Searching for ball
        while not flag_ball:
            flag_ball, ball_coords, ball_distance = finding('ball')      # (True/False), (x,y), distance
        print(f"result of finding ball: {ball_coords} and {ball_distance}")

        # Searching for basket
        while not flag_basket:          # DOESN'T WORKING, THERE SHOULD BE MASK AND THRESHOLDING
            flag_basket, basket_coords, basket_distance = finding('basket')      # (True/False), (x,y), distance
        print(f"result of finding ball: {basket_coords} and {basket_distance}")

        # Take ball and do several steps from ball holder
        
        flag_basket = False
        while not flag_evade:
            flag_evade = thinking_take(ball_coords, basket_coords)
        print("Atom avoided collision with holder")
        

        # Searching for basket again

        while not flag_basket:          # DOESN'T WORKING, THERE SHOULD BE MASK AND THRESHOLDING
            flag_basket, basket_coords, basket_distance = finding('basket')      # (True/False), (x,y), distance
        print(f"result of finding ball: {basket_coords} and {basket_distance}")

        # Approach basket first time

        flag_basket = False
        turn_to(basket_coords)    #DOESN'T WORK NOW
        # maybe do finding one more time
        go_to(1, basket_coords, basket_distance)  #DOESN'T WORK NOW

        # Searching for basket again

        while not flag_basket:          # DOESN'T WORKING, THERE SHOULD BE MASK AND THRESHOLDING
            flag_basket, basket_coords, basket_distance = finding('basket')      # (True/False), (x,y), distance
        print(f"result of finding ball: {basket_coords} and {basket_distance}")

        # Finally approach basket

        flag_basket = False
        turn_to(basket_coords)    #DOESN'T WORK NOW
        go_to(1, basket_coords, basket_distance)  #DOESN'T WORK NOW

        # put the ball into basketcase
        
        self.motion.play_Soft_Motion_Slot(name = 'basketball_throwing_ball')
        

        print("HE HIT THE BALL OR NO. I don't NO")
        while True:
            n = int(input())
            basketball.go_to_ball(1, n)
        '''

if __name__ == '__main__':
    default_button = 1
    basketballer = Basketball(default_button)
    print("BEGINING")
    basketballer.run()
    print("ENDING")