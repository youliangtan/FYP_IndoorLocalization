import numpy as np
import cv2
import RPi.GPIO as GPIO
import smbus

print "yes"
GPIO.setmode(GPIO.BOARD)
GPIO.setup(3, GPIO.IN)
GPIO.setup(5, GPIO.OUT)

print smbus

i2c = smbus.SMBus(0)
address = 0x68

test = i2c.read_byte_data(addres, 0x00)
print test
