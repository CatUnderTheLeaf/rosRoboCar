import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt
# %matplotlib inline

def show_all_images(images, columns = 4):
    plt.figure(figsize=(40, 20))
    i = 0
    for img in images:
        plt.subplot(len(images) / columns + 1, columns, i + 1)
        plt.imshow(img)
        i+=1
    plt.savefig('checkboards.png', bbox_inches='tight')
    
def show_images(orig_img, corr_img, xlabel1, xlabel2, cmap1 = None, cmap2 = None, save = False):
    plt.figure(figsize=(20,10))
    plt.subplot(1, 2, 1)    
    plt.imshow(orig_img, cmap = cmap1)
    plt.xlabel(xlabel1)
    plt.xticks([], [])
    plt.yticks([], [])
    plt.subplot(1, 2, 2)
    plt.imshow(corr_img, cmap = cmap2)
    plt.xlabel(xlabel2)
    plt.xticks([], [])
    plt.yticks([], [])
    if save:
        plt.savefig(xlabel2.replace(" ", "_") + '.png', bbox_inches='tight')
    plt.show()
    

# Functions connected with camera calibration and distortion coefficients

# Make a list of calibration images
images = glob.glob('*.jpg')

# array of chessboard images with corners
chess_board_images = []

def chessboard_corners(images):
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*8,3), np.float32)
    objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d points in real world space
    imgpoints = [] # 2d points in image plane.

    # Step through the list and search for chessboard corners
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (8,6),None)

        # If found, add object points, image points
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)

            # Draw and save images with corners
            img = cv2.drawChessboardCorners(img, (8,6), corners, ret)
            chess_board_images.append(img)
            
    return objpoints, imgpoints, gray.shape[::-1]

# get camera matrix and distortion coefficients 
def calibrate_camera(images):
    
    objpoints, imgpoints, shape = chessboard_corners(images)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, shape, None, None)
    
    P = np.zeros((3, 4), dtype=np.float64)
    ncm, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, shape, 0.0)
    for j in range(3):
        for i in range(3):
            P[j,i] = ncm[j, i]

    return mtx, dist, P

# apply camera matrix and distortion coefficients to undistort image
def cal_undistort(img, mtx, dist):    
    undist = cv2.undistort(img, mtx, dist, None, mtx)
    return undist





mtx, dist, P = calibrate_camera(images)
# show all passed calibration images with drawn corners
show_all_images(chess_board_images)
# print camera matrix and distortion coefficients 
print(mtx)
print(dist)
print(P)


# example of applied distortion correction
dist_test_img = plt.imread(images[1])
undist_test_img = cal_undistort(dist_test_img, mtx, dist)

show_images(dist_test_img, undist_test_img, xlabel1 = 'Distorted image', xlabel2 = 'Undistorted image', save=True)