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
# 初始化全局变量
PBL = 0

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

# 突出图像中的红色目标
def get_red():
    global image, image_red, image_green, image_blue
    image_red_only=image_red*2-image_blue-image_green
    image_red_only[image_red_only<0]=0
    return image_red_only

# 突出图像中的绿色目标
def get_green():
    global image, image_red, image_green, image_blue
    image_green_only=image_green*2-image_blue-image_red
    image_green_only[image_green_only<0]=0
    return image_green_only

# 突出图像中的蓝色目标
def get_blue():
    global image, image_red, image_green, image_blue
    image_blue_only=image_blue*2-image_blue-image_red
    image_blue_only[image_blue_only<0]=0
    return image_blue_only

# 突出目标颜色
def find_aim_color(aim_color):
    if aim_color == 'R':
        return get_red()
    if aim_color == 'G':
        return get_green()
    if aim_color == 'B':
        return get_blue()
    else:
        return get_blue()

# 定位物料位置的函数
def locate_aim_material(aim_image,image):
    dis_error, order=100,'K+000X+000Y+000'
    pass

# 定位目标位置的函数
def locate_aim_position(aim_image,image):
    dis_error, order=100,'K+000X+000Y+000'
    pass

# 外界大循环保证程序报错时依旧可以继续运行
while True:
    try:
        while True:
            PBL = ser_32.read(4)
            PBL=PBL.decode('utf-8')
            # 等待STM32发送控制指令给我，执行具体的任务，这里并不需要双线程，也不需要记录上位机
            if PBL == 'SCAN':
                # 扫描二维码
                pass
            # 此时是定位指令
            if PBL(0) == 'F':
                # 定义目标颜色
                aim_color=PBL(2)
                # 如果为物料定位指令
                if PBL(1) == 'M':
                    # 定义误差标准
                    dis_error = 100
                    while dis_error>10:
                        # 获取图像
                        get_image()
                        # 突出目标颜色
                        aim_image = find_aim_color(aim_color)
                        # 定位目标位置
                        dis_error, order=locate_aim_material(aim_image,image)
                        pass
                    pass
                # 此时是位置定位指令
                if PBL(1) == 'P':
                    # 定义误差标准
                    dis_error = 100
                    while dis_error>10:
                        # 获取图像
                        get_image()
                        # 突出目标颜色
                        aim_image = find_aim_color(aim_color)
                        # 定位目标位置
                        dis_error, order=locate_aim_position(aim_image,image)
                        pass
                    pass
            # 读取STM32的指令
            pass
    except:
        pass



