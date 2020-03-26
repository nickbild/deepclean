import Jetson.GPIO as GPIO
import time
from subprocess import call
import cv2
import numpy as np


img_width = 1024
img_height = 768
output_pin = 22
GPIO.setmode(GPIO.BOARD)

GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)

cmd = "scp pi@192.168.1.170:/home/pi/deepclean/pi.jpg ."


while True:
    # Retrieve image from Raspberry Pi.
    GPIO.output(output_pin, GPIO.HIGH)
    time.sleep(0.01)
    GPIO.output(output_pin, GPIO.LOW)
    call(cmd.split(" "))
    time.sleep(0.2)

    # Analyze stereo images.
    #imgL = cv2.imread('pi.jpg', 0)
    #imgR = cv2.imread('nano.jpg', 0)
    #stereo = cv2.StereoBM_create(numDisparities=32, blockSize=17)
    #disparity = stereo.compute(imgL,imgR)
    #cv2.imwrite('result.jpg', disparity)

