import RPi.GPIO as GPIO
import picamera
import time
import os


input_pin = 12
GPIO.setmode(GPIO.BOARD)
GPIO.setup(input_pin, GPIO.IN)

high = False
low = False

with picamera.PiCamera() as camera:
    time.sleep(2)
    
    camera.resolution = (1024,768)
    camera.rotation = 180
    camera.capture('dummy.jpg')
    
    while True:
        if GPIO.input(input_pin) == 1:
            high = True

        if high and GPIO.input(input_pin) == 0:
            low = True

        if high and low:
            camera.capture('next.jpg')
            high = False
            low = False
            os.rename('next.jpg', 'pi.jpg')

