import cv2
import numpy as np
import serial
import time
import threading

# 打开摄像机和端口，不轻易运行
cap=cv2.VideoCapture(0)
ser_stm32 = serial.Serial('/dev/ttyAMA0', 921600)
ser_BT = serial.Serial('/dev/ttyAMA3', 9600)

# 定义广义常亮
stop='V+000'
red_light_on='OKOK'
red_light_off='OFFF'
yellow_light_on='GOOD'
yellow_light_off='NONO'
buzzer_on='BEEP'
buzzer_off='QUIE'
error='E+000'
PBL=b'0'

# 在判断好数字后将决定前进的方向，输入为G±nn，L±nn，R±nn，到达终点后，发送OKOK
def send_order(order,ser):
    #print(order)
    encoded_data = order.encode()
    ser.write(encoded_data)

# 发送命令给stm32
def send_stm32(order):
    send_order(order,ser_stm32)

# 发送命令给车2
def send_BT(order):
    send_order(order,ser_BT)

# 获得stm32的信号
def get_stm32():
    hex_data = ser_stm32.read(1)
    return hex_data

# 获取2车信号
def get_BT():
    global PBL
    print('test')
    while True:
        PBL = ser_BT.read(1)
        #print('PBL ='+str(PBL))

# 这一步用于识别黑色条纹的位置，计算误差后传给stm32
def get_road_error(black):
    # 100:540
    global error,k
    d=np.argwhere(black[170:270,100:540] < 250)
    dis=np.mean(d,axis=0)-220
    k=dis[1]
    k=min(max(-500,k),500)
    x=0.8
    if k>=0:
        error='E'+'+'+str(int(int(k**(x))%1000/100))+str(int(int(k**(x))%100/10))+str(int(k**(x))%10)
    if k<0:
        error='E'+'-'+str(int(int((-k)**(x))%1000/100))+str(int(int((-k)**(x))%100/10))+str(int((-k)**(x))%10)
    return error,k

# 对图片进行分割操作, 分别返回黑色与红色的图片
def photo_process(cap):
    global image, red, black
    _,image=cap.read()
    red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
    black=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 对红色图片进行二值化，阈值为30
    _, red = cv2.threshold(red, 45, 255, cv2.THRESH_BINARY)
    _, black= cv2.threshold(black, 100, 255, cv2.THRESH_BINARY)

# 在路口识别分支，分支为0，正常为1
def get_cross(black):
    if np.sum(black[170:270,100:220] < 250)>2000:
        print('检测到路口标志')
        return 0
    else:
        return 1

# 监测停止标志
def get_stop(black):
    #print(np.sum(black[370:470,250:390]<250))
    if (np.sum(black[370:470,250:390]<250)>5000)&(np.sum(black[370:470,250:390]<250)<10000):
        print('检测到停止标志')
        return 0
    else:
        return 1

# 看前面有没有2车，有为1，没有为0
def see_car1(red):
    if np.sum(red[20:30,:] > 250)<50:
        return 0
    else:
        return 1

# 依据前方状态直行
def car_go_straight(road):
    error,_=get_road_error(road)
    send_order(error,ser_stm32)

# 立即停止
def car_stop():
    time.sleep(0.1)
    send_order(stop,ser_stm32)
    time.sleep(0.1)

speed=go1='V+065'
# 依据前方状态直行，并调整速度
def car_go_straight_adjust(red, black, see_car1_last):
    global speed
    send_order(error,ser_stm32)
    if (see_car1(red)==1)&(see_car1_last==0):
        send_order('NONO', ser_stm32)
        send_order('OKOK', ser_stm32)
        print('go1_low')
        send_stm32(go1_low)
        speed=go1_low
        see_car1_last=1
    if (see_car1(red)==0)&(see_car1_last==1):
        send_order('GOOD', ser_stm32)
        send_order('OFFF', ser_stm32)
        print('go1_high')
        #send_stm32(go1)
        send_stm32(go1_high)
        speed=go1_high
        see_car1_last=0
    if (see_car1(red)==0)&(see_car1_last==0):
        send_order('GOOD', ser_stm32)
        send_order('OFFF', ser_stm32)
        if speed==go1_low:
            print('go1_high')
            #send_stm32(go1)
            send_stm32(go1_high)
            see_car1_last=0
    if (see_car1(red)==1)&(see_car1_last==1):
        send_order('OKOK', ser_stm32)
        send_order('NONO', ser_stm32)
        if speed==go1_high:
            print('go1_low')
            #send_stm32(go1)
            send_stm32(go1_low)
            see_car1_last=1
    return see_car1_last

# 车往前冲
def car_rush():
    print('车直接往前冲')
    time.sleep(1.3)

# 车转弯
def car_turn():
    print('车强制往左转弯')
    send_stm32('E+030')
    time.sleep(1.3)

def get_error():
    # 更新误差
    while True:
        photo_process(cap)
        error,_=get_road_error(black)
        # cv2.imshow('black',black)
        # cv2.imshow('red',red)
        # cv2.imshow('image',image)
        # cv2.waitKey(1)
        # print(error)

get_error_threading = threading.Thread(target=get_error)
get_error_threading.start()
get_car1_threading = threading.Thread(target=get_BT)
get_car1_threading.start()
send_order('RSET', ser_stm32)
time.sleep(5)

PBL=b'0'
see_car1_last=0
# 保证程序一次出错后依旧可以持续运行
while True:
    # 一旦开机，反复检测题目序号
    while True:
        if PBL==b'1':
            go1='V+060'
            turn_time=10
            # 题目1速度0.3m/s
            time.sleep(0.05)
            send_stm32(go1)
            print('第一问的巡线，直接往前冲一段，然后一直巡迹')
            while get_stop(black):
                see_car1_last=car_go_straight_adjust(red, black, see_car1_last)
                time.sleep(0.005)
            print('检测到路口标志，开始冲')
            car_rush()
            print('第一圈，已经冲完，开始延时巡迹')
            for i in range(100):
                see_car1_last=car_go_straight_adjust(red, black, see_car1_last)
                time.sleep(0.005)
            print('第一圈，开始听消息巡迹')
            #send_order('OKOK', ser_stm32)
            while PBL!=b'0':
                #print('PBL='+str(PBL))
                see_car1_last=car_go_straight_adjust(red, black, see_car1_last)
                time.sleep(0.005)
            # 等待1车信号，等到就停下
            car_stop()
            PBL=0
            pass
        elif PBL==b'2':
            go1='V+050'
            go1_low='V+048'
            go1_high='V+052'
            turn_time=10
            # 题目1速度0.3m/s
            send_stm32(go1)
            time.sleep(0.05)
            send_stm32(go1)
            print('第2问的巡线')
            red, black=photo_process(cap)
            while get_stop(black):
                red, black=photo_process(cap)
                see_car1_last=car_go_straight_adjust(red, black, see_car1_last)
            print('检测到路口标志，开始冲')
            car_rush()
            print('第1圈，已经冲完，开始延时巡迹')
            red, black=photo_process(cap)
            for i in range(100):
                red, black=photo_process(cap)
                see_car1_last=car_go_straight_adjust(red, black, see_car1_last)
            print('第1圈，开始检测巡迹')            
            while get_stop(black):
                red, black=photo_process(cap)
                see_car1_last=car_go_straight_adjust(red, black, see_car1_last)
            print('检测到路口标志，开始冲')            
            car_rush()
            print('第2圈，已经冲完，开始延时巡迹')
            red, black=photo_process(cap)
            for i in range(100):
                red, black=photo_process(cap)
                see_car1_last=car_go_straight_adjust(red, black, see_car1_last)
            print('第2圈，开始听消息巡迹')
            red, black=photo_process(cap)
            while PBL!=0:
                red, black=photo_process(cap)
                see_car1_last=car_go_straight_adjust(red, black, see_car1_last)
            # 等待1车信号，等到就停下
            car_stop()
            PBL=0
            pass
        elif PBL==b'3':
            # 题目3速度≥0.3m/s
            go3='V+30'
            pass
        elif PBL==b'4':
            # 题目4速度1.0m/s
            go4='V+30'
            pass
        else:
            car_stop()
            print('没有具体题目，继续等待')
            print('PBL='+str(PBL))
            time.sleep(0.1)
            pass