import cv2
import numpy as np
import serial
import numpy as np
import time
import threading

# 打开摄像机和端口，不轻易运行
cap=cv2.VideoCapture(0)
ser_stm32 = serial.Serial('/dev/ttyAMA0', 921600)
ser_car1 = serial.Serial('/dev/ttyAMA3', 9600)

PBL=0
def get_BT():
    while True:
        global PBL
        hex_data = ser_car1.read(1)
        PBL=int(hex_data)
        print('并行逻辑中PBL的值'+str(PBL))
        time.sleep(0.1)

t1 = threading.Thread(target=get_BT)
t1.start()

for i in range(1000):
    print(1)
    time.sleep(2)
