import Jetson.GPIO as GPIO
import time
from subprocess import call
import cv2
import numpy as np
import pyopenpose as op
import os


img_width = 1024
img_height = 768
output_pins = [22, 24]
params = dict()
params["model_folder"] = "/home/nick/software/openpose/openpose/models/"

GPIO.setmode(GPIO.BOARD)

for output_pin in output_pins:
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)

cmds = ["scp pi@192.168.1.170:/home/pi/deepclean/pi.jpg pi1.jpg", 
        "scp pi@192.168.1.171:/home/pi/deepclean/pi.jpg pi2.jpg"]
FNULL = open(os.devnull, 'w')

opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()

# Load stereo calibration data.
calibration = np.load("camera_calibration/stereo_calibration.npz", allow_pickle=False)
imageSize = tuple(calibration["imageSize"])
leftMapX = calibration["leftMapX"]
leftMapY = calibration["leftMapY"]
leftROI = tuple(calibration["leftROI"])
rightMapX = calibration["rightMapX"]
rightMapY = calibration["rightMapY"]
rightROI = tuple(calibration["rightROI"])

while True:
    # Capture images from Raspberry Pis.
    for output_pin in output_pins:
        GPIO.output(output_pin, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(output_pin, GPIO.LOW)

    time.sleep(0.25)

    # Retrieve images from Raspberry Pis.
    for cmd in cmds:
        call(cmd.split(" "), stdout=FNULL)

    # Rectify/analyze stereo images.
    imgL = cv2.imread('pi2.jpg')
    imgR = cv2.imread('pi1.jpg')
    #stereo = cv2.StereoBM_create(numDisparities=128, blockSize=15)
    stereo = cv2.StereoSGBM_create(numDisparities=16, blockSize=1)
    
    fixedL = cv2.remap(imgL, leftMapX, leftMapY, cv2.INTER_LINEAR)
    fixedR = cv2.remap(imgR, rightMapX, rightMapY, cv2.INTER_LINEAR)
    
    grayL = cv2.cvtColor(fixedL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(fixedR, cv2.COLOR_BGR2GRAY)

    stereo.setMinDisparity(-2)
    stereo.setSpeckleRange(2)
    stereo.setSpeckleWindowSize(5)

    disparity = stereo.compute(grayL, grayR)

    #cv2.imwrite('left.jpg', grayL)
    #cv2.imwrite('right.jpg', grayR)

    # Determine body position.
    datum = op.Datum()
    datum.cvInputData = fixedL
    opWrapper.emplaceAndPop([datum])

    try:
        # Wrist location.
        #print("R wrist: {}".format(datum.poseKeypoints[0][4])) # Right wrist [x, y, score].
        #print("L wrist: {}".format(datum.poseKeypoints[0][7])) # Left wrist [x, y, score].

        # Raw / mean disparity in region.
        if int(datum.poseKeypoints[0][4][0]) > 0:
            print("Right ++++")
            #print(disparity[int(datum.poseKeypoints[0][4][0])][int(datum.poseKeypoints[0][4][1])])
            disparityLwrist = disparity[ int(datum.poseKeypoints[0][4][0])-20:int(datum.poseKeypoints[0][4][0])+20, int(datum.poseKeypoints[0][4][1])-20: int(datum.poseKeypoints[0][4][1])+20 ]
            print(disparityLwrist.mean())
        
        if int(datum.poseKeypoints[0][7][0]) > 0:
            #print("Left ----")
            #print(disparity[int(datum.poseKeypoints[0][7][0])][int(datum.poseKeypoints[0][7][1])])
            disparityRwrist = disparity[ int(datum.poseKeypoints[0][7][0])-20:int(datum.poseKeypoints[0][7][0])+20, int(datum.poseKeypoints[0][7][1])-20: int(datum.poseKeypoints[0][7][1])+20 ]
            #print(disparityRwrist.mean())

        # Draw circles on image.
        disparity = cv2.circle(disparity, (int(datum.poseKeypoints[0][4][0]), int(datum.poseKeypoints[0][4][1])), 20, (255, 0, 0), 2)
        disparity = cv2.circle(disparity, (int(datum.poseKeypoints[0][7][0]), int(datum.poseKeypoints[0][7][1])), 20, (255, 0, 0), 2)

    except:
        print("No human detected.")

    # View disparity map.
    cv2.imwrite('result_distance.jpg', disparity)

    # View annotated pose image.
    # newImage = datum.cvOutputData[:, :, :]
    # cv2.imwrite("result_pose.jpg", newImage)

