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
    print('order:'+order)
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

# 初始化摄像头，初始化图像---------------------------------------------------------------
'''
for i in range(30):
    #time.sleep(2)
    _,image=cap.read()
    road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
    # 对图片进行二值化，阈值为30
    _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
    print('first'+str(i))
    get_road_error(road)
# '''

# send_order('V+00',ser)
# send_order('V+00',ser)
send_order('OKOK',ser)



# send_order('L+90',ser)

# send_order('R+90',ser)

# time.sleep(4)

# send_order('V+00',ser)

# send_order('L+90',ser)
# send_order('L+90',ser)




# a=['12', 'we', 'qqq']
# for i in a:
#     print(i, end='')

# '''
# juzi=[]
# while True:
#     b=str(get_stm32feed(ser))
#     a=b[2]
#     juzi.append(a)
#     # print(a)
#     if(a=='q'):
#         # print(juzi)
#         for i in juzi:
#             print(i, end='')
#         print('\n')
#         juzi=[]
# '''

# 以延时的方式进行前进
# for i in range(50):
# 以循环的方式进行前进
# while True:
#     _,image=cap.read()
#     road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
#     # 对图片进行二值化，阈值为30
#     _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
#     # print(get_cross(road))
#     error,k=get_road_error(road)
#     send_order(error,ser)
#     # print(np.sum(road[420:430,:] > 250))