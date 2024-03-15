# import numpy as np
# import cv2 as cv
# import glob, os
# # termination criteria
# criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# l_cell, w_cell = 10, 7
# # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# objp = np.zeros((w_cell*l_cell,3), np.float32)
# objp[:,:2] = np.mgrid[0:l_cell,0:w_cell].T.reshape(-1,2)
# # Arrays to store object points and image points from all the images.
# objpoints = []                                                                  # 3d point in real world space
# imgpoints = []                                                                  # 2d points in image plane.
# rootFolder =  "camera_calibrator"
# imageFormat = ".jpg"
# imageFolder = "{}/images".format(rootFolder)
# OutputFolder =  "{}/output".format(rootFolder)
# if not  os.path.isdir(rootFolder):
#     os.mkdir(rootFolder)
# if not  os.path.isdir(imageFolder):
#     os.mkdir(imageFolder)
# if not  os.path.isdir(OutputFolder):
#     os.mkdir(OutputFolder)
# images = glob.glob("{}/*{}".format(imageFolder, imageFormat))
# print(images)
# for fname in images:
#     img = cv.imread(fname)
#     gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
#     # Find the chess board corners
#     ret, corners = cv.findChessboardCorners(gray, (l_cell,w_cell), None)
#     # If found, add object points, image points (after refining them)
#     if ret == True:
#         objpoints.append(objp)
#         corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
#         imgpoints.append(corners2)
#         # Draw and display the corners
#         cv.drawChessboardCorners(img, (l_cell,w_cell), corners2, ret)
#         cv.imshow('img', img)
#         cv.waitKey(200)


# print(type(imgpoints))
# print(imgpoints)
# np.save("{}/imgpoints.npy".format(OutputFolder), imgpoints)
# # np.savetxt("{}/imgpoints.txt".format(OutputFolder), imgpoints, "%.4e")
# cv.destroyAllWindows()









# rosrun camera_calibration cameracalibrator.py --size 10x7 --square .024 image:=/home/shihab/omobot_js/camera_calibrator/images camera:=/camera


import cv2 
import numpy as np 
import os 
import glob 
  
  
# Define the dimensions of checkerboard 
l_cell, w_cell = 10, 7
CHECKERBOARD = (w_cell, l_cell) 

curCam = "Cam1"
curPathDict = {
   "Cam0": "/left" , 
   "Cam1": "/rght",
   "None": " copy"
}

rootFolder =  "/home/shihab/omobot_js/camera_calibrator"
imageFormat = ".jpg"
imageFolder = "{}/images{}".format(rootFolder, curPathDict[curCam])
OutputFolder =  "{}/output{}".format(rootFolder, curPathDict[curCam])
# stop the iteration when specified 
# accuracy, epsilon, is reached or 
# specified number of iterations are completed. 
criteria = (cv2.TERM_CRITERIA_EPS + 
            cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) 
  
  
# Vector for 3D points 
threedpoints = [] 
  
# Vector for 2D points 
twodpoints = [] 
  
  
#  3D points real world coordinates 
objectp3d = np.zeros((1, CHECKERBOARD[0]  
                      * CHECKERBOARD[1],  
                      3), np.float32) 
objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 
                               0:CHECKERBOARD[1]].T.reshape(-1, 2) 
prev_img_shape = None
  
  
# Extracting path of individual image stored 
# in a given directory. Since no path is 
# specified, it will take current directory 
# jpg files alone 
print(imageFolder)
print(os.listdir(imageFolder))
images = glob.glob("{}/*{}".format(imageFolder, imageFormat))
print(images)
for filename in images: 
    image = cv2.imread(filename) 
    grayColor = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
  
    # Find the chess board corners 
    # If desired number of corners are 
    # found in the image then ret = true 
    ret, corners = cv2.findChessboardCorners( 
                    grayColor, CHECKERBOARD,  
                    cv2.CALIB_CB_ADAPTIVE_THRESH  
                    + cv2.CALIB_CB_FAST_CHECK + 
                    cv2.CALIB_CB_NORMALIZE_IMAGE) 
  
    # If desired number of corners can be detected then, 
    # refine the pixel coordinates and display 
    # them on the images of checker board 
    if ret == True: 
        threedpoints.append(objectp3d) 
  
        # Refining pixel coordinates 
        # for given 2d points. 
        corners2 = cv2.cornerSubPix( 
            grayColor, corners, (11, 11), (-1, -1), criteria) 
  
        twodpoints.append(corners2) 
  
        # Draw and display the corners 
        image = cv2.drawChessboardCorners(image,  
                                          CHECKERBOARD,  
                                          corners2, ret) 
  
    cv2.imshow('img', image) 
    cv2.waitKey(200) 
  
cv2.destroyAllWindows() 
  
h, w = image.shape[:2] 
  

# Perform camera calibration by 
# passing the value of above found out 3D points (threedpoints) 
# and its corresponding pixel coordinates of the 
# detected corners (twodpoints) 
ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera( 
    threedpoints, twodpoints, grayColor.shape[::-1], None, None) 
  
  
# Displaying required output 


print(" Camera matrix {}:".format(curCam)) 
print(matrix) 
  
print("\n Distortion coefficient {}:".format(curCam)) 
print(distortion) 
  
print("\n Rotation Vectors {}:".format(curCam)) 
print(r_vecs) 
  
print("\n Translation Vectors {}:".format(curCam)) 
print(t_vecs)

np.save("{}/twodpoints{}.npy".format(OutputFolder, curCam), twodpoints)
np.save("{}/matrix{}.npy".format(OutputFolder, curCam), matrix)
np.save("{}/distortion{}.npy".format(OutputFolder, curCam), distortion)
np.save("{}/r_vecs{}.npy".format(OutputFolder, curCam), r_vecs)
np.save("{}/t_vecs{}.npy".format(OutputFolder, curCam), t_vecs)