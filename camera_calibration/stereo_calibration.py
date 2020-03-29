import numpy as np
from matplotlib import pyplot as plt
import cv2
import glob
import sys
import os


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
        #os.remove(fname)
        #os.remove(fnameR)

# sys.exit(0)

# Calibrate individual images.
retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(objpoints, imgpointsL, grayL.shape[::-1], None, None)
retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(objpoints, imgpointsR, grayR.shape[::-1], None, None)

h, w = grayL.shape[:2]
newcameramtxL, roiL = cv2.getOptimalNewCameraMatrix(mtxL, distL, (w,h), 1, (w,h))
newcameramtxR, roiR = cv2.getOptimalNewCameraMatrix(mtxR, distR, (w,h), 1, (w,h))

# Rectify image pair.
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

# Stereo calibration.
retval, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(objpoints, imgpointsL, imgpointsR, mtxL, distL, mtxR, distR, (w,h), None, None, None, None, cv2.CALIB_FIX_INTRINSIC, criteria)
print(retval)

# Stereo rectification.
(leftRectification, rightRectification, leftProjection, rightProjection,
        dispartityToDepthMap, leftROI, rightROI) = cv2.stereoRectify(
                mtxL, distL,
                mtxR, distR,
                (w,h), R, T,
                None, None, None, None, None,
                cv2.CALIB_ZERO_DISPARITY, 0.25)

leftMapX, leftMapY = cv2.initUndistortRectifyMap(
        mtxL, distL, leftRectification,
        leftProjection, (w,h), cv2.CV_32FC1)

rightMapX, rightMapY = cv2.initUndistortRectifyMap(
        mtxR, distR, rightRectification,
        rightProjection, (w,h), cv2.CV_32FC1)

np.savez_compressed("stereo_calibration.npz", imageSize=(w,h),
        leftMapX=leftMapX, leftMapY=leftMapY, leftROI=leftROI,
        rightMapX=rightMapX, rightMapY=rightMapY, rightROI=rightROI)
