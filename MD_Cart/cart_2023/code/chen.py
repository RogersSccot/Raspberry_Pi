import cv2
import numpy as np
import serial
import time
import matplotlib.pyplot as plt

capture = cv2.VideoCapture(0)
_,image=capture.read()
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
while True:
    cv2.imshow('image',hsv)
    cv2.waitKey(1)



