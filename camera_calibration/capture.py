import Jetson.GPIO as GPIO
import time
from subprocess import call
import cv2
import os


output_pins = [22, 24]

GPIO.setmode(GPIO.BOARD)

for output_pin in output_pins:
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)

cmds = ["scp pi@192.168.1.170:/home/pi/deepclean/pi.jpg pi1.jpg", 
        "scp pi@192.168.1.171:/home/pi/deepclean/pi.jpg pi2.jpg"]

cnt = 0
while True:
    print(cnt)
    time.sleep(1)

    # Capture images from Raspberry Pis.
    for output_pin in output_pins:
        GPIO.output(output_pin, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(output_pin, GPIO.LOW)

    time.sleep(1)

    # Retrieve images from Raspberry Pis.
    for cmd in cmds:
        call(cmd.split(" "))

    os.rename("pi1.jpg", "img/pi1_{}.jpg".format(cnt))
    os.rename("pi2.jpg", "img/pi2_{}.jpg".format(cnt))
    cnt += 1

