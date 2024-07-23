import cv2
import numpy as np
import serial
import numpy as np
import time

# 打开摄像机和端口，不轻易运行
cap=cv2.VideoCapture(0)
ser_stm32 = serial.Serial('/dev/ttyAMA0', 921600)
ser_car2 = serial.Serial('/dev/ttyAMA3', 9600)

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

# 发送命令给stm32
def send_stm32(order):
    send_order(order,ser_stm32)

# 发送命令给车2
def send_car2(order):
    send_order(order,ser_car2)

# 获得stm32的信号
def get_stm32():
    hex_data = ser_stm32.read(1)
    return hex_data

# 获取2车信号
def get_car2():
    hex_data = ser_car2.read(1)
    return hex_data

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
    _, black= cv2.threshold(black, 177, 255, cv2.THRESH_BINARY)
    return red, black

# 在路口识别分支，分支为0，正常为1
def get_cross(road):
    if np.sum(road[420:430,:] > 250)<10:
        return 0
    else:
        return 1
    
# 监测停止标志
def get_stop(road):
    if np.sum(road[420:430,:] > 250)<10:
        return 0
    else:
        return 1

# 依据前方状态直行
def car_go_straight(road):
    error,_=get_road_error(road)
    send_order(error,ser_stm32)

# 看前面有没有2车，有为1，没有为0
def see_car2(red):
    if np.sum(red[420:430,:] > 250)<10:
        return 0
    else:
        return 1

# 立即停止
def car_stop():
    time.sleep(0.1)
    send_order(stop,ser_stm32)
    time.sleep(0.1)

# 题目1速度0.3m/s
go1_low='V+28'
go1_high='V+32'
see_car2_last=0
# 依据前方状态直行，并调整速度
def car_go_straight_adjust(red, black, see_car2_last):
    error,_=get_road_error(black)
    send_order(error,ser_stm32)
    if (see_car2(red)==1)&(see_car2_last==0):
        send_car2(go1_low)
        see_car2_last=1
    if (see_car2(red)==0)&(see_car2_last==1):
        send_car2(go1_high)
        see_car2_last=0
    return see_car2_last

# 车往前冲
def car_rush():
    for i in range(10):
        send_stm32('E+000')

# 车转弯
def car_turn():
    for i in range(10):
        send_stm32('E+030')

# 保证程序一次出错后依旧可以持续运行
while True:
    # 一旦开机，反复检测题目序号
    while True:
        PBL=int(get_stm32())
        # 在接受到stm32的消息后，第一时间发送给2车
        send_car2(str(PBL))
        if PBL==1:
            # 题目1速度0.3m/s
            go1='V+30'
            send_stm32(go1)
            # 此时开始巡迹，车处在1,2段
            while get_cross():
                red, black=photo_process(cap)
                car_go_straight(black)
            # 此时检测到路口，车处在2处， 继续直行
            car_rush()
            # 度过路口后， 车处在2,3段
            while get_cross():
                red, black=photo_process(cap)
                car_go_straight(black)
            # 此时检测到路口，车处在3处， 开始转弯
            car_rush()
            # 转弯后，车处在3,4段
            while get_cross():
                red, black=photo_process(cap)
                car_go_straight(black)
            # 此时检测到路口，车处在4处， 开始转弯
            car_turn()
            # 转弯后，车处在4,5段
            while get_cross():
                red, black=photo_process(cap)
                car_go_straight(black)
            # 此时检测到路口，车处在5处， 继续直行
            car_rush()
            # 转弯后，车处在5,6段
            while get_cross():
                red, black=photo_process(cap)
                car_go_straight(black)
            # 此时检测到路口，车处在6处， 开始转弯
            car_turn()
            # 转弯后，车处在6,1段
            while get_cross():
                red, black=photo_process(cap)
                car_go_straight(black)
            # 此时检测到路口，车处在1处， 开始转弯
            car_turn()
            # 转弯后，车处在1,2段
            while get_stop():
                red, black=photo_process(cap)
                car_go_straight(black)
            # 此时已经结束，，给2车发信号，车停下
            send_car2('0')
            car_stop()
            pass
        elif PBL==2:
            # 题目2速度0.5m/s
            go2='V+30'
            pass
        elif PBL==3:
            # 题目3速度≥0.3m/s
            go3='V+30'
            pass
        elif PBL==4:
            # 题目4速度1.0m/s
            go4='V+30'
            pass
        else:
            break