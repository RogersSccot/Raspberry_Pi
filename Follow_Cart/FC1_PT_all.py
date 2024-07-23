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
    print(order)
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

# 获取蓝牙信号
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
    if (np.sum(black[370:470,250:390]<250)>7000)&(np.sum(black[370:470,250:390]<250)<10000):
        print('检测到停止标志')
        return 0
    else:
        return 1

# 看前面有没有2车，有为1，没有为0
see_red=1
def see_car1(red):
    global see_red
    see_red1=np.sum(red > 250)
    if see_red1>1000:
        see_red=see_red1
    else:
        see_red=see_red
    return see_red

# 依据前方状态直行
def car_go_straight(road):
    error,_=get_road_error(road)
    send_order(error,ser_stm32)

# 立即停止
def car_stop():
    time.sleep(0.1)
    send_order('E+000V+000',ser_stm32)
    time.sleep(0.1)

go_v=65
go1='E+000V+062'
vp=0.0001
vi=0
vd=0
# 依据前方状态直行，并调整速度
def speed_fix():
    see_red=see_car1(red)
    speed_v=go_v-(see_red-54000)*vp
    speed_fixed='V+'+str(int(int(speed_v)%1000/100))+str(int(int(speed_v)%100/10))+str(int(speed_v)%10)
    print('speed_fixed='+speed_fixed)
    return speed_fixed

# 车往前冲
def car_rush():
    print('车直接往前冲')
    send_stm32('E-020V+065')
    time.sleep(1.5)

# 车转弯
def car_turn():
    print('车强制往左转弯')
    send_stm32('E+030V+063')
    time.sleep(1.3)

def get_error():
    # 更新误差
    while True:
        photo_process(cap)
        error,_=get_road_error(black)
        #cv2.imshow('black',black)
        #cv2.imshow('red',red)
        #cv2.imshow('image',image)
        #cv2.waitKey(1)
        #print(error)

get_error_threading = threading.Thread(target=get_error)
get_error_threading.start()
send_order('RSET', ser_stm32)
time.sleep(3)

PBL=b'2'
# 保证程序一次出错后依旧可以持续运行
while True:
    # 一旦开机，反复检测题目序号
    while True:
        #PBL=get_stm32()
        # 在接受到stm32的消息后，第一时间发送给2车
        if PBL==b'1':
            #######################################################
            # 题目1速度0.3m/s
            time.sleep(0.05)
            send_BT('1')
            time.sleep(0.05)
            go1='V+063'
            send_stm32('E+000'+go1)
            print('第一问的巡线，直接往前冲一段，然后一直巡迹')
            car_rush()
            print('第一圈，已经冲完，开始延时巡迹')
            for i in range(2500):
                send_order(error+go1,ser_stm32)
                time.sleep(0.005)
            print('第一圈，开始监测巡迹')
            while get_stop(black):
                send_order(error+go1,ser_stm32)
                time.sleep(0.005)
            print('此时已经结束，给2车发信号，车停下')
            time.sleep(0.1)
            send_BT('0')
            car_stop()
            PBL=0
            pass
        elif PBL==b'2':
            #########################################################
            # 题目2速度0.5m/s
            time.sleep(0.05)
            send_BT('2')
            go1='V+080'
            print('第2问的巡线')
            car_rush()
            send_stm32('E+000'+go1)
            print('第1圈外圈，已经冲完，开始延时巡迹')
            for i in range(100):
                send_order(error+go1,ser_stm32)
                time.sleep(0.005)
            print('第1圈，开始监测巡迹')
            while get_stop(black):
                send_order(error+go1,ser_stm32)
                time.sleep(0.005)
            print('此时第一圈结束，开始第二圈')
            ##########################################################
            car_rush()
            print('第2圈，已经冲完，开始延时巡迹')
            for i in range(100):
                send_order(error+go1,ser_stm32)
                time.sleep(0.005)
            print('第2圈，开始监测巡迹')
            while get_stop(black):
                send_order(error+go1,ser_stm32)
                time.sleep(0.005)
            print('此时第2圈结束')
            send_BT('0')
            car_stop()
            PBL=0            
            pass
        elif PBL==b'3':
            #######################################################
            # 题目2速度0.5m/s
            time.sleep(0.05)
            send_BT('3')
            go1='E+000V+063'
            send_stm32(go1)
            print('第3问的巡线')
            car_rush()
            print('第1圈外圈，已经冲完，开始延时巡迹')
            for i in range(100):
                send_order(error+go1,ser_stm32)
                time.sleep(0.005)
            print('第1圈，开始监测巡迹')
            while get_stop(black):
                send_order(error+go1,ser_stm32)
                time.sleep(0.005)
            print('此时第一圈结束，开始第二圈')
            ########################################################
            go_v=63
            car_rush()
            print('第2圈，已经冲完，开始延时巡迹')
            for i in range(100):
                send_order(error+go1,ser_stm32)
                time.sleep(0.005)
            print('第2圈，开始监测巡迹')
            while get_stop(black):
                send_order(error+speed_fix(),ser_stm32)
                time.sleep(0.005)
            print('此时第2圈结束, 开始第三圈')
            ########################################################
            print('第3圈，强行转弯，开始延时巡迹')
            go_v=63
            car_turn()
            for i in range(100):
                send_order(error+go1,ser_stm32)
                time.sleep(0.005)
            print('第2圈，开始监测巡迹')
            while get_stop(black):
                send_order(error+speed_fix(),ser_stm32)
                time.sleep(0.005)            
            send_BT('0')
            car_stop()
            PBL=0            
            pass
        elif PBL==b'4':
            # 题目4速度1.0m/s
            go4='V+30'
            pass
        elif PBL==b't':
            # 测试程序
            print('看到的红色='+str(see_car1(red)))    
        else:
            car_stop()
            print('没有具体题目，继续等待')
            time.sleep(0.1)
            break