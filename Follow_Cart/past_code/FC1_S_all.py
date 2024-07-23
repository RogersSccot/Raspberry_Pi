import cv2
import numpy as np
import serial
import numpy as np
import time
import threading

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
buzzer_on='BEEP'
buzzer_off='QUIE'
PBL=0

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
    global PBL
    hex_data = ser_car2.read(1)
    PBL=int(hex_data)
    return PBL

# 这一步用于识别黑色条纹的位置，计算误差后传给stm32
def get_road_error(line):
    # 100:540
    d=np.argwhere(line[170:270,100:540] < 250)
    dis=np.mean(d,axis=0)-220
    k=dis[1]
    k=min(max(-500,k),500)
    x=0.8
    if k>=0:
        error='E'+'+'+str(int(int(k**(x))%1000/100))+str(int(int(k**(x))%100/10))+str(int(k**(x))%10)
    if k<0:
        error='E'+'-'+str(int(int((-k)**(x))%1000/100))+str(int(int((-k)**(x))%100/10))+str(int((-k)**(x))%10)
    #print(error)
    return error,k

# 对图片进行分割操作, 分别返回黑色与红色的图片
def photo_process(cap):
    _,image=cap.read()
    red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
    black=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 对红色图片进行二值化，阈值为30
    _, red = cv2.threshold(red, 45, 255, cv2.THRESH_BINARY)
    _, black= cv2.threshold(black, 100, 255, cv2.THRESH_BINARY)
    return red, black

# 在路口识别分支，分支为0，正常为1
def get_cross(line):
    if np.sum(line[170:270,100:220] < 250)>2000:
        print('检测到路口标志')
        return 0
    else:
        return 1

# 监测停止标志
def get_stop(line):
    if np.sum(line[370:470,250:390]<250)>5000:
        print('检测到停止标志')
        return 0
    else:
        return 1

# 看前面有没有2车，有为1，没有为0
def see_car2(red):
    if np.sum(red[20:30,:] > 250)<10:
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

# 题目1速度0.3m/s
go1_low='V+048'
go1_high='V+052'
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
    print('车直接往前冲')
    for i in range(15):
        send_stm32('E+000')

# 车转弯
def car_turn():
    print('车强制往左转弯')
    for i in range(15):
        send_stm32('E+030')

PBL=0
# 保证程序一次出错后依旧可以持续运行
while True:
    # 一旦开机，反复检测题目序号
    while True:
        PBL=int(get_stm32())
        # 在接受到stm32的消息后，第一时间发送给2车
        send_car2(str(PBL))
        if PBL==1:
            # 题目1速度0.3m/s
            go1='V+050'
            go1_low='V+048'
            go1_high='V+052'
            turn_time=10
            # 题目1速度0.3m/s
            send_stm32(go1)
            time.sleep(0.05)
            send_stm32(go1)
            print('第一问的巡线，直接往前冲一段，然后一直巡迹')
            car_rush()
            print('第一圈，已经冲完，开始延时巡迹')
            red, black=photo_process(cap)
            for i in range(100):
                red, black=photo_process(cap)
                car_go_straight(black)
            print('第一圈，开始监测巡迹')
            red, black=photo_process(cap)
            while get_stop(black):
                red, black=photo_process(cap)
                car_go_straight(black)
            print('# 此时已经结束，给2车发信号，车停下')
            send_car2('0')
            car_stop()
            PBL=0
            pass
        elif PBL==2:
            # 题目1速度0.3m/s
            go1='V+050'
            go1_low='V+048'
            go1_high='V+052'
            turn_time=10
            # 题目1速度0.3m/s
            send_stm32(go1)
            time.sleep(0.05)
            send_stm32(go1)
            print('第2问的巡线')
            car_rush()
            print('第1圈，已经冲完，开始延时巡迹')
            red, black=photo_process(cap)
            for i in range(100):
                red, black=photo_process(cap)
                car_go_straight(black)
            print('第1圈，开始监测巡迹')
            red, black=photo_process(cap)
            while get_stop(black):
                red, black=photo_process(cap)
                car_go_straight(black)
            print('此时第一圈结束，开始第二圈')
            car_rush()
            print('第2圈，已经冲完，开始延时巡迹')
            red, black=photo_process(cap)
            for i in range(100):
                red, black=photo_process(cap)
                car_go_straight(black)
            print('第2圈，开始监测巡迹')
            red, black=photo_process(cap)
            while get_stop(black):
                red, black=photo_process(cap)
                car_go_straight(black)
            print('此时第2圈结束')
            send_car2('0')
            car_stop()
            PBL=0            
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