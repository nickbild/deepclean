import Jetson.GPIO as GPIO
import time
from subprocess import call
import cv2


output_pins = [22, 24]

GPIO.setmode(GPIO.BOARD)

for output_pin in output_pins:
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)

cmds = ["scp pi@192.168.1.170:/home/pi/deepclean/pi.jpg pi1.jpg", 
        "scp pi@192.168.1.171:/home/pi/deepclean/pi.jpg pi2.jpg"]

cnt = 0
while True:
    # Capture images from Raspberry Pis.
    for output_pin in output_pins:
        GPIO.output(output_pin, GPIO.HIGH)
        time.sleep(0.0001)
        GPIO.output(output_pin, GPIO.LOW)

    # Retrieve images from Raspberry Pis.
    for cmd in cmds:
        call(cmd.split(" "))

    os.rename("pi1", "img/pi1_{}.jpg".format(cnt))
    os.rename("pi2", "img/pi2_{}.jpg".format(cnt))
    cnt += 1

