import cv2
import numpy as np
import serial
import time
import threading

# 打开摄像头，占用内存大，不轻易运行
capture=cv2.VideoCapture(0)
# 定义中心点坐标
center_point=np.array([21.42,32.59])
red_point_send='X21.42Y32.59'
ser_32 = serial.Serial('/dev/ttyAMA0', 921600)
ser_screen=serial.Serial('/dev/ttyAMA3', 115200)

# 发送命令
def send_order(order):
    print(order)
    encoded_data = order.encode()
    ser_32.write(encoded_data)

# 获取蓝牙信号
def get_PBL():
    global PBL
    print('读取题目信息')
    while True:
        PBL = ser_screen.read(1)
        print('PBL ='+str(PBL))

# 突出图像中的红色目标
def get_red():
    global image, image_red, image_green, image_blue
    image_red_only=image_red*2-image_blue-image_green
    image_red_only[image_red_only<0]=0
    image_red_only_photo=image_red_only.astype(np.uint8)
    return image_red_only_photo, image_red_only

# 获取球坐标
def get_ball(image_red_only):
    global red_point
    ex1_red=cv2.threshold(image_red_only, 200, 255, cv2.THRESH_BINARY)[1]
    red_point=np.where(ex1_red>200)
    red_point=np.mean(red_point,axis=1)
    return red_point

# 在某一图像的指定位置添加图案
kernel = np.array([
[0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0],
[0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0],
[0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0],
[0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0],
[0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0],
[0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0],
[0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0],
[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
[0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0],
[0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0],
[0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0],
[0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0],
[0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0],
[0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0],
[0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0],
])*255
def draw_image(image_gray_to_draw, kernel_point):
    try:
        image_gray_to_draw[int(kernel_point[0]-11):int(kernel_point[0]+11),int(kernel_point[1]-11):int(kernel_point[1]+11)]=kernel
    except:
        image_gray_to_draw[240-11:240+11,320-11:320+11]=kernel
    return image_gray_to_draw

def red_point_char(red_point):
    red_point_x=int(red_point[0]*10)
    red_point_y=int(red_point[1]*10)
    red_point_send='X'+str(int(red_point_x/1000))+str(int((red_point_x/100)%10))+'.'+str(int((red_point_x/10)%10))+str(int((red_point_x)%10))+'Y'+str(int(red_point_y/1000))+str(int((red_point_y/100)%10))+'.'+str(int((red_point_y/10)%10))+str(int((red_point_y)%10))
    return red_point_send

# 获取并处理图像
def get_image():
    while True:
        try:
            # 开全局变量处理，分理处红绿蓝
            global image, image_red, image_green, image_blue, red_point, red_point_send
            # 从摄像头获取图像
            _,image=capture.read()
            # image=cv2.imread('1.jpg')
            image_red=image[:,:,2].astype(np.float32)
            image_green=image[:,:,1].astype(np.float32)
            image_blue=image[:,:,0].astype(np.float32)
            image_red_only_photo,image_red_only=get_red()
            red_point=get_ball(image_red_only)
            red_point_send=red_point_char(red_point)
            send_order(red_point_send)
            print(red_point_send)
            time.sleep(0.007)
            #cv2.imshow('image_red_only',image_red_only_photo)
            #image_draw=draw_image(image_red_only_photo,red_point)
            #cv2.imshow('image_draw',image_draw)
            #cv2.waitKey(1)
        except:
            pass

# 获取图像单独开进程
get_image_threading = threading.Thread(target=get_image)
get_image_threading.start()
get_PBL_threading = threading.Thread(target=get_PBL)
get_PBL_threading.start()

p1='M12.89B22.81'
p2='M12.33B40.55'
p3='M31.68B23.52'
p4='M29.70B41.69'
p5='M21.42B32.59'

while True:
    try:
        while True:
            if PBL==b'1':
                send_order(p2)
            if PBL==b'2':
                send_order(p1)
                time.sleep(6)
                send_order(p5)
            if PBL==b'3':
                send_order(p1)
                time.sleep(6)
                send_order(p5)   
                time.sleep(6)
                send_order(p4) 
            if PBL==b'4':
                send_order(p1)
                time.sleep(6)
                send_order(p4)                                
            
    except:
        pass

