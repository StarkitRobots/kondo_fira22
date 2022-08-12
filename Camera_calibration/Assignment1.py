import numpy as np
import cv2
import glob
import yaml
import os


#import pathlab

def saveCoefficients(mtx, dist, path):
    """ Save the camera matrix and the distortion coefficients to given path/file. """
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    cv_file.write("camera_matrix", mtx)
    cv_file.write("dist_coeff", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()


def loadCoefficients(path):
    """ Loads camera matrix and distortion coefficients. """
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("camera_matrix").mat()
    dist_matrix = cv_file.getNode("dist_coeff").mat()

    # Debug: print the values
    # print("camera_matrix : ", camera_matrix.tolist())
    # print("dist_matrix : ", dist_matrix.tolist())

    cv_file.release()
    return [camera_matrix, dist_matrix]





corner_x=8 # number of chessboard corner in x direction
corner_y=6 # number of chessboard corner in y direction

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((corner_y*corner_x,3), np.float32)
objp[:,:2] = np.mgrid[0:corner_x,0:corner_y].T.reshape(-1,2)

# Arrays to store object points and image points from a ll the images.
objpoints = [] # 3d point in real world space
jpegpoints = [] # 2d points in image plane.

source_path = "/home/pi/MIPI_Camera_old/RPI/python/Camera_calibration" #ini untuk source image kita simpan kat mane??
#print(os.getcwd())
print('image found :',len(os.listdir(source_path)))

images = [source_path + '/' + f for f in glob.glob('*.png')]

#images = glob.glob('D:\Image Vision')


# path = 'results'
# pathlib.Path(path).mkdir(parents=True, exist_ok=True)

found = 0
for fname in images: # here, 10 can be changed to whatever number you like to choose
    png_image = cv2.imread(fname) # capture frame by frame
    cv2.imshow('png_image', png_image)
    cv2.waitKey(500)
    print(fname)
    gray = cv2.cvtColor(png_image, cv2.COLOR_BGRA2GRAY)
    
    # find the chess noard corners
    ret, corners = cv2.findChessboardCorners(gray, (corner_x,corner_y), None)
    # if found, ass object points, image points (after refining them)
    if ret == True:
        
        objpoints.append(objp) #Certainly, every loop objp is the same in 3D
        corners2 = cv2.cornerSubPix(gray,corners,(20,5),(-1,-5),criteria)
        jpegpoints.append(corners2)
        # Draw and display the corners
        png_image = cv2.drawChessboardCorners(png_image, (corner_x,corner_y), corners2, ret)
        found += 1
        cv2.imshow('chessboard', png_image)
        cv2.waitKey(10)
        # if you want to save images with dtected corners
      
        
print("number of images used for calibration: ", found)

 # when everything done, release the capture
# cap.release()
# cv2.destroyAllWindows()

#calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, jpegpoints, gray.shape[::-1], None, None, flags = cv2.CALIB_RATIONAL_MODEL)
#ret, mtx, dist, rvecs, tvecs = cv2.fisheye.calibrate(objpoints, jpegpoints, gray.shape[::-1], None, None)
# transforms the matrix distortion coefficients to writeable lists
data= {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}
print(mtx)
print(dist)
# and save it to a file
with open("calibration_matrix.yaml", "w")as f:
    yaml.dump(data, f)
saveCoefficients(mtx, dist, "mtx.yaml")
#undistort image

for fname in images: # here, 10 can be changed to whatever number you like to choose
     #print(fname)
     png_image = cv2.imread(fname) # Capture frame-by-frame
     #cv2.imshow('png_image', png_image)
     #cv2.waitkey(500)
     h, w = png_image.shape[:2]
     #print('png_image.shape', h, w)
     newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx, dist , (w,h), 1, (w,h))
     #print(newcameramtx)
     #undistort
     dst = cv2.undistort(png_image, mtx, dist, None, newcameramtx)
     cv2.imshow('undistort', dst)
     cv2.waitKey(10)
     
     # crop the image
     x, y, w, h = roi
     print('roi =', roi)
     dst = dst[y:y+h, x:x+w]
     #cv2.imshow('calibration.png',dst)
     cv2.imshow('undistort2', dst)
     cv2.waitKey(10)
     
cv2.destroyAllWindows() 
