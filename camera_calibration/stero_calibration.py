import numpy as np
from matplotlib import pyplot as plt
import cv2
import glob
import sys


# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpointsL = [] # 2d points in image plane.
imgpointsR = [] # 2d points in image plane.

images = glob.glob("img/good/pi2_*.jpg")

for fname in images:
    imgL = cv2.imread(fname)
    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)

    fnameR = fname.replace("pi2_", "pi1_")
    imgR = cv2.imread(fnameR)
    grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    retL, cornersL = cv2.findChessboardCorners(grayL, (7,6), None)
    retR, cornersR = cv2.findChessboardCorners(grayR, (7,6), None)

    # If found, add object points, image points (after refining them)
    if retL and retR:
        print(fname)

        objpoints.append(objp)

        cornersL2 = cv2.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criteria)
        imgpointsL.append(cornersL)

        cornersR2 = cv2.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), criteria)
        imgpointsR.append(cornersR)

        # Draw and display the corners
        #cv2.drawChessboardCorners(imgL, (7,6), cornersL2, retL)
        #cv2.imwrite('{}.corners.jpg'.format(fname), imgL)

        #cv2.drawChessboardCorners(imgR, (7,6), cornersR2, retR)
        #cv2.imwrite('{}.corners.jpg'.format(fnameR), imgR)
    else:
        print("NO: {}".format(fname))





#sys.exit(0)


# Calibrate individual images.
retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(objpoints, imgpointsL, grayL.shape[::-1], None, None)
retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(objpoints, imgpointsR, grayR.shape[::-1], None, None)

h, w = grayL.shape[:2]
newcameramtxL, roiL = cv2.getOptimalNewCameraMatrix(mtxL, distL, (w,h), 1, (w,h))
newcameramtxR, roiR = cv2.getOptimalNewCameraMatrix(mtxR, distR, (w,h), 1, (w,h))

# Rectify image.
img = cv2.imread("img/good/pi2_25_good.jpg")
dst = cv2.undistort(img, mtxL, distL, None, newcameramtxL)
x, y, w, h = roiL
print("{} {} {} {}".format(x, y, w, h))
dst = dst[y:y+h, x:x+w]
cv2.imwrite("img/pi2_25_rectified.jpg", dst)


img = cv2.imread("img/good/pi1_25_good.jpg")
dst = cv2.undistort(img, mtxR, distR, None, newcameramtxR)
print("{} {} {} {}".format(x, y, w, h))
dst = dst[y:y+h, x:x+w]
cv2.imwrite("img/pi1_25_rectified.jpg", dst)







#
# for fname in images:
#     img = cv2.imread(fname)
#
#     # undistort
#     dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
#     # crop the image
#     x, y, w, h = roi
#     dst = dst[y:y+h, x:x+w]
#     cv2.imshow('img', dst)
#     cv2.waitKey(500)
#
# cv2.destroyAllWindows()
#
# # Now you can store the camera matrix and distortion coefficients using write functions in Numpy (np.savez, np.savetxt etc) for future uses.
#
#
# #retval,cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(objpointsL, imgpointsL, imgpointsR, (w,h))
#
# # Stereo rectification
# retval, _, _, _, _, R, T, E, F=cv2.stereoCalibrate(all_3d_points,  all_left_corners, all_right_corners, (im.shape[1],im.shape[0]),mtx,dist,mtx,dist,flags=cv2.cv.CV_CALIB_FIX_INTRINSIC)
#
# # Stereo calibration
#
# R1=cv2.cv.fromarray(np.zeros((3,3))) #output 3x3 matrix
# R2=cv2.cv.fromarray(np.zeros((3,3))) #output 3x3 matrix
# P1=cv2.cv.fromarray(np.zeros((3,4))) #output 3x4 matrix
# P2=cv2.cv.fromarray(np.zeros((3,4))) #output 3x4 matrix
#
# roi1,roi2=cv2.cv.StereoRectify(cv2.cv.fromarray(mtx), #intrinsic parameters of the first camera
#    cv2.cv.fromarray(mtx), #intrinsic parameters of the second camera
#    cv2.cv.fromarray(dist), #distortion parameters of the first camera
#    cv2.cv.fromarray(dist), #distortion parameters of the second camera
#    (left_im.shape[1],left_im.shape[0]), #image dimensions
#    cv2.cv.fromarray(R), #Rotation matrix between first and second cameras (returned by cv2.stereoCalibrate)
#    cv2.cv.fromarray(T), #Translation vector between coordinate systems of the cameras (returned by cv2.stereoCalibrate)
#    R1,R2,P1,P2) #last 4 parameters point to inizialized output variables
#
# R1=np.array(R1) #convert output back to numpy format
# R2=np.array(R2)
# P1=np.array(P1)
# P2=np.array(P2)
#
# map1_x,map1_y=cv2.initUndistortRectifyMap(mtx, dist, R1, P1, (left_im.shape[1],left_im.shape[0]), cv2.cv.CV_32FC1)
# map2_x,map2_y=cv2.initUndistortRectifyMap(mtx, dist, R2, P2, (left_im.shape[1],left_im.shape[0]), cv2.cv.CV_32FC1)
#
# im_left=cv2.imread('left07.jpg')
# im_right=cv2.imread('right07.jpg')
#
# im_left_remapped=cv2.remap(im_left,map1_x,map1_y,cv2.INTER_CUBIC)
# im_right_remapped=cv2.remap(im_right,map2_x,map2_y,cv2.INTER_CUBIC)
#
# out=np.hstack((im_left_remapped,im_right_remapped))
#
# plt.figure(figsize=(10,4))
# plt.imshow(out[...,::-1])
# plt.show()
