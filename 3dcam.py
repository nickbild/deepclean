import Jetson.GPIO as GPIO
import time
from subprocess import call
import cv2
import numpy as np
import pyopenpose as op


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

opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()

while True:
    # Capture images from Raspberry Pis.
    for output_pin in output_pins:
        GPIO.output(output_pin, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(output_pin, GPIO.LOW)

    # Retrieve images from Raspberry Pis.
    for cmd in cmds:
        call(cmd.split(" "))

    # Analyze stereo images.
    imgL = cv2.imread('pi1.jpg', 0)
    imgR = cv2.imread('pi2.jpg', 0)
    stereo = cv2.StereoBM_create(numDisparities=32, blockSize=17)
    disparity = stereo.compute(imgL,imgR)
    cv2.imwrite('result_distance.jpg', disparity)

    # Determine body position.
    imgLcolor = cv2.imread('pi1.jpg')
    datum = op.Datum()
    datum.cvInputData = imgLcolor
    opWrapper.emplaceAndPop([datum])
    newImage = datum.cvOutputData[:, :, :]
    cv2.imwrite("result_pose.jpg", newImage)

