# 导入必要的函数库
import cv2
import numpy as np
import math
import serial
import threading
import time

# 打开摄像头，占用内存大，不轻易运行
capture=cv2.VideoCapture(0)

# 打开串口
ser_32 = serial.Serial('/dev/ttyAMA0', 921600)

PBL=0
aim_point22=0
aim_point23=0
last_car_point='C00'
car_point='C00'
car_center=[0,0]
grid_str=['0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F']
grid_int=['00','01','02','03','04','05','06','07','08','09','10','11','12','13','14','15']
M=np.load('M.npy')
# 我们需要将我们读取到的图像映射成一个平整的图像
ad=cv2.imread('grid.jpg')
ad_grey=cv2.cvtColor(ad, cv2.COLOR_BGR2GRAY)

# 开启单独进程，获取题目信号
'''
通信协议
1.收到Axx，表示2.1问的目标点
2.收到Bxx，表示2.2问的目标点
3.收到Cxx，表示2.3问的目标点
'''
def get_PBL():
    global PBL, aim_point22, aim_point23
    print('读取题目信息')
    while True:
        PBL = ser_32.read(3)
        print('mail ='+str(PBL))
        if PBL[0]==66:
            aim_point22=(PBL[1]-48)*10+(PBL[2]-48)
        elif PBL[0]==67:
            aim_point23=(PBL[1]-48)*10+(PBL[2]-48)

get_PBL_threading = threading.Thread(target=get_PBL)
get_PBL_threading.start()

# 发送命令，通用函数
'''
通信协议
1.发送Lxx，Rxx，Fxx将会直接反馈给小车
2.发送Pxx，等一系列点用于指定路径
3.发送Cxx，将用于指定小车位置
4.发送Mxx，将用于修改地图布局
'''
def send_order(order, ser):
    print('order='+order)
    encoded_order = order.encode()
    ser.write(encoded_order)

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
    image_red_only[image_red_only<100]=0
    image_red_only[image_red_only>100]=255
    return image_red_only

# 颜色识别找车
def find_car_red(image_gray):
    global car_center
    try:
        d1=np.argwhere(image_gray>100)
        red1=np.mean(d1,axis=0)
        red2=np.array([red1[1],red1[0],1])
        red3=np.dot(M,red2)
        car_center=[red3[1]/red3[2],red3[0]/red3[2]]
    except:
        car_center=car_center
    return car_center

# 定位函数，定位车在哪个格子里
def locate_car(car_center):
    # 首先找到小车在16宫格内的坐标
    car_center_M=car_center
    car_index=int(car_center_M[0]/242)*4+int(car_center_M[1]/242)
    car_point='C'+str(int(car_index/10))+str(car_index%10)
    return car_point

# 找到小车的点坐标
def get_car_point():
    get_image()
    red_only=get_red()
    car_center=find_car_red(red_only)
    car_point=locate_car(car_center)
    return car_point

# 做一个映射变换
def image_trans(image_to_trans):
    image_after_trans = cv2.warpPerspective(image_to_trans,M,(ad.shape[1],ad.shape[0]))
    image_after_trans=cv2.cvtColor(image_after_trans, cv2.COLOR_BGR2GRAY)
    _,image_after_trans=cv2.threshold(image_after_trans, 127, 255, cv2.THRESH_BINARY)
    return image_after_trans

get_image()
map_origin=image_trans(image)

while True:
    try:
        if PBL==0:
            time.sleep(0.1)
        if PBL==b'P21':
            # 对应题目的2.1问，首先要确立路径点，然后给串口屏发定位
            while PBL!=b'000':
                try:
                    car_point=get_car_point()
                except:
                    pass
                if car_point!=last_car_point:
                    send_order(car_point,ser_32)
                    last_car_point=car_point
        if PBL==b'P22':
            get_image()
            map_coverd=image_trans(image)
            black_cover=(map_origin-map_coverd)
            grid_coverd_str=[]
            grid_coverd_int=[]
            for i in range(16):
                x=int(i/4)*242
                y=int(i%4)*242
                b1=np.sum(black_cover[x:x+242,y:y+242]>100)
                print(f'{i}:'+str(b1))
                if (b1>3000):
                    grid_coverd_str.append('MBB')
                else:
                    grid_coverd_str.append('M'+grid_int[i])
                    grid_coverd_int.append(i)
            # 向stm32发送地图信息
            if aim_point22!=0:
                # 开始路径规划
                pass
            pass
        else:
            time.sleep(0.1)
    except:
        PBL=0
        last_car_point='C00'
        aim_point22=0
        aim_point23=0
        time.sleep(0.1)

