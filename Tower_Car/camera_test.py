import cv2
import numpy as np
import serial
import cv2
import numpy as np
import time
from Vision_Net import FastestDet

# 打开摄像机和端口，不轻易运行
# ser = serial.Serial('/dev/ttyAMA0', 115200)
cap=cv2.VideoCapture(1)
loo_global=np.zeros((640,480,3),dtype=np.uint8)
deep = FastestDet(drawOutput=True)

# 获取照片数字 
def get_photo_number(frame):
    out = deep.detect(frame)
    num_position=[]
    num=[]
    for i in range(len(out)):
        num_position.append(out[i][0])
        num.append(int(out[i][1]))
    return  num_position,num

s=1
for i in range(10):
    try:
        frame=cv2.imread(f"C:/Users/SaLuo/Desktop/data-1/val/{s}.jpg")
        out = deep.detect(frame)
        cv2.imwrite(f'out{s}.jpg', frame)
        s=s+10
    except:
        break