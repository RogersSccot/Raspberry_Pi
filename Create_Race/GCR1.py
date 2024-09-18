import numpy as np
import cv2
import math
import matplotlib
import serial
import time
from Vision_Net import FastestDet

# 本代码用于工创赛视觉部分，此为第一版本2024.9.12
# 对于完整的视觉代码，主要需要解决如下问题
'''
核心任务及通信协议
1.识别二维码
SCAN
2.识别物料本身的精确位置，并传递误差
(find the aim material and transport the location error)
FMRE(红色物料),FMGE(绿色物料),FMBE(蓝色物料)
3.识别物料放置区精确位置，并传递误差，并传递误差
(find the aim position and transport the location error)
FPRE(红色位置),FPGE(绿色位置),FPBE(蓝色位置)
'''
# 打开摄像头，占用内存大，不轻易运行
capture=cv2.VideoCapture(0)
# 视觉神经网络先初始化，备用
loo_global=np.zeros((640,480,3),dtype=np.uint8)
deep = FastestDet(drawOutput=True)
# 打开串口
ser_32 = serial.Serial('/dev/ttyAMA0', 921600)

# 发送命令的指令
def send_order(order):
    print('order='+order)
    encoded_order = order.encode()
    ser_32.write(encoded_order)

# 获取消息，通用函数
def get_mail(ser):
    encoded_mail = ser.read(1)
    print('mail='+encoded_mail)

# 获取并处理图像
def get_image():
    # 开全局变量处理，分理处红绿蓝
    global image, image_red, image_green, image_blue
    # 从摄像头获取图像
    _,image=capture.read()
    image_red=image[:,:,2].astype(np.float32)
    image_green=image[:,:,1].astype(np.float32)
    image_blue=image[:,:,0].astype(np.float32)
    #cv2.imshow('image',image)
    #cv2.waitKey(1)

# 外界大循环保证程序报错时依旧可以继续运行
while True:
    try:
        while True:
            # 等待STM32发送控制指令给我，执行具体的任务，这里并不需要双线程，也不需要记录上位机

            pass
    except:
        pass



