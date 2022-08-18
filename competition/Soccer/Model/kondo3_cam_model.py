import numpy as np
import cv2

class CameraModel:
    def __init__(self, glob):
        # super().__init__(params, "model")
        self.do_undistortions = False

        self.camera_matrix, self.distortion = self.loadCoefficients("../../kondo_fira22/Camera_calibration/mtx.yaml")
        self.focal_length_x = self.camera_matrix[1][1]     #add it to PARAMS!!!
        self.focal_length_y = self.camera_matrix[1][1]
        self.center_x = self.camera_matrix[0][2]      #add it to PARAMS!!!
        self.center_y = self.camera_matrix[1][2]      #add it to PARAMS!!!
        #self.distortion = np.array([-0.090947, 0.011789, -0.001821, -0.004004, 0.000000], dtype=np.float32) #add it to PARAMS!!!
        #self.camera_matrix = np.array(
            #[[self.focal_length, 0., self.center_x], 
            #[0., self.focal_length, self.center_y], 
            #[0., 0., 1.]], dtype=np.float32)

    def loadCoefficients(self, path):
        """ Loads camera matrix and distortion coefficients. """
        # FILE_STORAGE_READ
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        camera_matrix = cv_file.getNode("camera_matrix").mat()
        dist_matrix = cv_file.getNode("dist_coeff").mat()
        cv_file.release()
        return camera_matrix, dist_matrix

    def get_view_vector_from_pixel(self, pixel_x, pixel_y):
        """
        Getting view vector from camera to pixel 
        Args:
            pixel_x (int): coords in pixels
            pixel_y (int): coords in pixels
        Returns:
            turple: Normalized view vector from camera to pixel
        """
        
        if self.do_undistortions:
             undistort_points = self.undistortPoints(pixel_x, pixel_y)
             print(undistort_points)
             pixel_x, pixel_y = undistort_points[0], undistort_points[1]
        delta_pixel_x = pixel_x - self.center_x
        delta_pixel_y = pixel_y - self.center_y
        #length = np.sqrt(delta_pixel_x**2 + delta_pixel_y**2 + focal_length**2)

        #return (delta_pixel_x/length, delta_pixel_y/length, focal_length/length)
        return (delta_pixel_x/self.focal_length_x, delta_pixel_y/self.focal_length_y, 1.0)

    def undistortPoints(self, x, y):
        points = np.array([x, y], dtype=np.float32)
        return cv2.undistortPoints(points, self.camera_matrix, self.distortion)[0][0]