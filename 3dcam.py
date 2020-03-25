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


def gstreamer_pipeline (capture_width=3280, capture_height=2464, display_width=img_width, display_height=img_height, framerate=21, flip_method=0) :
    return ('nvarguscamerasrc ! '
    'video/x-raw(memory:NVMM), '
    'width=(int)%d, height=(int)%d, '
    'format=(string)NV12, framerate=(fraction)%d/1 ! '
    'nvvidconv flip-method=%d ! '
    'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
    'videoconvert ! '
    'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))


cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

while True:
    # Capture image.
    ret_val, img_in = cap.read()
    cv2.imwrite("nano.jpg", img_in)

    # Retrieve image from Raspberry Pi.
    GPIO.output(output_pin, GPIO.HIGH)
    time.sleep(0.01)
    GPIO.output(output_pin, GPIO.LOW)
    call(cmd.split(" "))
    time.sleep(0.2)

    # Analyze stereo images.
    imgL = cv2.imread('pi.jpg', 0)
    imgR = cv2.imread('nano.jpg', 0)
    stereo = cv2.StereoBM_create(numDisparities=32, blockSize=17)
    disparity = stereo.compute(imgL,imgR)
    cv2.imwrite('result.jpg', disparity)

