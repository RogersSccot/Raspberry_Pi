import cv2
import numpy as np
import serial
import cv2
import numpy as np
import time
from Vision_Net import FastestDet

# 打开摄像机和端口，不轻易运行
ser = serial.Serial('/dev/ttyAMA0', 115200)
cap=cv2.VideoCapture(0)
loo_global=np.zeros((640,480,3),dtype=np.uint8)
deep = FastestDet(drawOutput=True)
def get_photo_number(frame):
    out = deep.detect(frame)
    num_position=[]
    num=[]
    for i in range(len(out)):
        num_position.append(out[i][0])
        num.append(int(out[i][1]))
    return  num_position,num

# 在判断好数字后将决定前进的方向，输入为G±nn，L±nn，R±nn，到达终点后，发送OKOK
def send_order(order,ser):
    print(order)
    encoded_data = order.encode()
    ser.write(encoded_data)

# 获得stm32的反馈，当stm32检测到已到达一个cornor时，或者一段命令已经执行完后，发送1给树莓派
def get_stm32feed(ser):
    hex_data = ser.read(1)
    return hex_data

# 这一步用于识别红色条纹的位置，计算误差后传给stm32
def get_road_error(road):
    d=np.argwhere(road[[450,350,250],:] > 250)
    dis=np.mean(d,axis=0)-320
    k=dis[1]
    k=k/3
    error='E+00'
    k=min(max(-100,k),100)
    x=0.8
    if k>=0:
        error='E'+'-'+str(int(int(k**(x))%100/10))+str(int(k**(x))%10)
    if k<0:
        error='E'+'+'+str(int(int((-k)**(x))%100/10))+str(int((-k)**(x))%10)
    return error,k

# 在路口前(识别数字时)发2，过路口后发(要转弯时)3，到路的尽头后发4，正常发1
def get_cross(road):
    if np.sum(road[10:20,:] > 250)>300:
        return 2
    if np.sum(road[420:430,:] > 250)>450:
        return 3
    if np.sum(road[420:430,:] > 250)<10:
        return 4
    else:
        return 1

# 监测压力传感器监测药物状态，有药为1，无药为0
def get_drug(ser):
    hex_data =  int(ser.read(1))
    return hex_data

send_order('RSET', ser)

# 初始化摄像头，初始化图像
print(1)
for i in range(30):
    #time.sleep(2)
    _,image=cap.read()
    road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
    # 对图片进行二值化，阈值为30
    _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
    print('first'+str(i))
    get_road_error(road)
print(2)
# 首先识别数字
'''
given_num=0
while given_num not in [1,2,3,4,5,6,7,8]:
    _,image=cap.read()
    _,given_num=get_photo_number(image)
    given_num=given_num[0]
    print('given_number='+str(given_num))
print('given_number='+str(given_num))
print('begin')
send_order('OKOK',ser)
time.sleep(1)
send_order('OFFF',ser)
# '''

# 监测药的状态

left='L+30'
right='R+30'
back='L+30'
go='V+30'
left_time=0.64
right_time=0.72
back_time=1.3
stop_time=2

given_num=1

if given_num==1:
    #----------------------------------------------------------------当数字为1的时候------------------------------------------------------
    print('数字为1,行程开始')
    send_order(go,ser)
    _,image=cap.read()
    road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
    # 对图片进行二值化，阈值为30
    _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
    # 当还没有经过路口时，继续直行
    a=(np.sum(road[420:430,:] > 250))
    print(a)

    while a<1200:
        _,image=cap.read()
        road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
        # 对图片进行二值化，阈值为30
        _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
        a=(np.sum(road[420:430,:] > 250))
        print(a)
        error,k=get_road_error(road)
        send_order(error,ser)
    time.sleep(0.2)
    # 到达路口，先停下
    
    send_order('V+00',ser)
    send_order('V+00',ser)
    send_order('V+00',ser)
    time.sleep(stop_time)

    # 现在开始向左转弯
    send_order(left,ser)
    time.sleep(left_time)
    send_order('V+00',ser)
    time.sleep(stop_time)

    send_order(go,ser)
    # 当还没有经过路口时，继续直行
    while get_cross(road)!=4:
        _,image=cap.read()
        road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
        # 对图片进行二值化，阈值为30
        _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
        # print(get_cross(road))
        error,k=get_road_error(road)
        send_order(error,ser)

    # 到达路的尽头，先停顿一下
    send_order('V+00',ser)
    send_order('V+00',ser)
    send_order('V+00',ser)
    send_order('OKOK',ser)

    time.sleep(3)
    # 监测药的状态
    # drug='y'
    # while drug!='n':
    #     drug=get_drug()
    # 当药物没有时
    send_order('OFFF',ser)
    
    # '''
    # 现在开始掉头回去------------------------------------------------------行程过半--------------------------------------------------------
    send_order(back,ser)
    time.sleep(back_time)
    send_order('V+00',ser)

    send_order('RSET', ser)
    time.sleep(2)
    ser.close()
    ser = serial.Serial('/dev/ttyAMA0', 115200)

    # '''
    send_order(go,ser)
    _,image=cap.read()
    road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
    # 对图片进行二值化，阈值为30
    _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)

    a=(np.sum(road[420:430,:] > 250))
    print(a)
    while a<1200:
        _,image=cap.read()
        road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
        # 对图片进行二值化，阈值为30
        _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
        a=(np.sum(road[420:430,:] > 250))
        print(a)
        error,k=get_road_error(road)
        send_order(error,ser)
    # time.sleep(0.1)

    # '''
    # 到达路口，先停下
    send_order('V+00',ser)
    send_order('V+00',ser)
    send_order('V+00',ser)
    time.sleep(stop_time)

    # '''
    # 现在开始向右转弯
    send_order(right,ser)
    time.sleep(right_time)
    send_order('V+00',ser)
    time.sleep(stop_time)
    # '''

    send_order(go,ser)

    _,image=cap.read()
    road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
    # 对图片进行二值化，阈值为30
    _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)

    # 当还没有经过路口时，继续直行
    while get_cross(road)!=4:
        _,image=cap.read()
        road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
        # 对图片进行二值化，阈值为30
        _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
        # print(get_cross(road))
        error,k=get_road_error(road)
        send_order(error,ser)
    time.sleep(1)
    # 到达路的尽头，先停顿一下
    send_order('V+00',ser)
    send_order('V+00',ser)
    send_order('V+00',ser)
    send_order('OKOK',ser)
    time.sleep(stop_time)
    send_order('OFFF',ser)
    # '''

if given_num==2:
    #---------------------------------------------------------------当数字为2的时候-----------------------------------------------------------
    print('数字为2,行程开始')
    send_order(go,ser)
    _,image=cap.read()
    road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
    # 对图片进行二值化，阈值为30
    _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
    # 当还没有经过路口时，继续直行
    while get_cross(road)!=3:
        _,image=cap.read()
        road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
        # 对图片进行二值化，阈值为30
        _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
        error,k=get_road_error(road)
        send_order(error,ser)
    # 到达路口，先停下
    time.sleep(0.1)
    send_order('V+00',ser)

    # 现在开始向右转弯
    send_order(right,ser)
    time.sleep(right_time)
    time.sleep(0.1)
    send_order('V+00',ser)

    send_order(go,ser)
    # 当还没有经过路口时，继续直行
    while get_cross(road)!=4:
        _,image=cap.read()
        road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
        # 对图片进行二值化，阈值为30
        _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
        # print(get_cross(road))
        error,k=get_road_error(road)
        send_order(error,ser)

    # 到达路的尽头，先停顿一下
    send_order('V+00',ser)
    time.sleep(1)

    send_order('OKOK',ser)
    # 监测药的状态
    drug='y'
    while drug!='n':
        drug=get_drug()
    # 当药物没有时
    send_order('OFFF',ser)

    # 现在开始掉头回去--------------------------------------------------------行程过半------------------------------------------------------------------
    send_order(back,ser)
    time.sleep(back_time)

    send_order(go,ser)
    _,image=cap.read()
    road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
    # 对图片进行二值化，阈值为30
    _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
    # 当还没有经过路口时，继续直行
    while get_cross(road)!=3:
        _,image=cap.read()
        road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
        # 对图片进行二值化，阈值为30
        _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
        error,k=get_road_error(road)
        send_order(error,ser)
    # 到达路口，先停下
    time.sleep(0.1)
    send_order('V+00',ser)

    # 现在开始向左转弯
    send_order(left,ser)
    time.sleep(left_time)
    time.sleep(0.1)
    send_order('V+00',ser)

    send_order(go,ser)
    # 当还没有经过路口时，继续直行
    while get_cross(road)!=4:
        _,image=cap.read()
        road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
        # 对图片进行二值化，阈值为30
        _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
        # print(get_cross(road))
        error,k=get_road_error(road)
        send_order(error,ser)

    # 到达路的尽头，先停顿一下
    send_order('V+00',ser)
    time.sleep(1)
    send_order('OKOK',ser)
