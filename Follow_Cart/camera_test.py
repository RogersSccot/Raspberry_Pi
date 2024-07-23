import cv2
import numpy as np
import serial
import numpy as np
import time

# 打开摄像机和端口，不轻易运行
# cap=cv2.VideoCapture(0)
# ser_stm32 = serial.Serial('/dev/ttyAMA0', 921600)
# ser_car2 = serial.Serial('/dev/ttyAMA3', 9600)

# 定义广义常亮
stop='V+000'
red_light_on='OKOK'
red_light_off='OFFF'
yellow_light_on='GOOD'
yellow_light_off='NONO'

# 在判断好数字后将决定前进的方向，输入为G±nn，L±nn，R±nn，到达终点后，发送OKOK
def send_order(order,ser):
    print(order)
    encoded_data = order.encode()
    ser.write(encoded_data)

# # 获取2车信号
# def get_car2():
#     hex_data = ser_car2.read(1)
#     return hex_data

# 这一步用于识别黑色条纹的位置，计算误差后传给stm32
def get_road_error(road):
    d=np.argwhere(road[[450,350,250],:] > 250)
    dis=np.mean(d,axis=0)-320
    k=dis[1]
    k=k
    error='E+000'
    k=min(max(-100,k),100)
    x=0.8
    if k>=0:
        error='E'+'-'+str(int(int(k**(x))%1000/100))+str(int(int(k**(x))%100/10))+str(int(k**(x))%10)
    if k<0:
        error='E'+'+'+str(int(int((-k)**(x))%1000/100))+str(int(int((-k)**(x))%100/10))+str(int((-k)**(x))%10)
    return error,k

# 对图片进行分割操作, 分别返回黑色与红色的图片
def photo_process(cap):
    _,image=cap.read()
    red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
    black = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
    # 对红色图片进行二值化，阈值为30
    _, red = cv2.threshold(red, 45, 255, cv2.THRESH_BINARY)
    _, black= cv2.threshold(black, 100, 255, cv2.THRESH_BINARY)
    return red, black

# 1.测试路口弯道识别
# 在路口识别分支，有分支为0，正常为1
def get_cross(road):
    if np.sum(road[:,0:250] > 250)>300:
        return 0
    else:
        return 1

# 2.测试停车标志识别
# 监测停止标志, 有为0，没有为1
def get_stop(road):
    if (np.sum(road[:,0:250] > 250)>100)&(np.sum(road[:,389:639] > 250)>100):
        return 0
    else:
        return 1

# 3.测试车辆识别
# 看前面有没有2车，有为1，没有为0
def see_car2(red):
    if np.sum(red[0:240,200:439] > 250)>200:
        return 1
    else:
        return 0

cap1=cv2.VideoCapture(1)
cap2=cv2.VideoCapture(2)
while True:
    _, image1=cap1.read()
    _, image2=cap2.read()
    cv2.imshow('image1',image1)
    cv2.imshow('image2',image2)
    red1, black1 = photo_process(cap1)
    red2, black2 = photo_process(cap2)
    cv2.imshow('black1',black1)
    cv2.imshow('red1',red1)
    cv2.waitKey(1)
    print('1车是否看到弯道   ：'+str(np.sum(black1[:,0:250] > 250)))
    # print('2车是否看到弯道   ：'+str(get_cross(black2)))
    # print('1车是否看到停车   ：'+str(get_stop(black1)))
    # print('2车是否看到停车   ：'+str(get_stop(black2)))
    # print('1车是否看到车2    ：'+str(see_car2(red1)))
    # print('2车是否看到车1    ：'+str(see_car2(red2)))
    time.sleep(0.1)