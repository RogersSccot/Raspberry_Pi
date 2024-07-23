import cv2
import numpy as np
import serial
import numpy as np
import time
from Vision_Net import FastestDet
import gc

# 打开摄像机和端口，不轻易运行
cap=cv2.VideoCapture(0)
loo_global=np.zeros((640,480,3),dtype=np.uint8)
deep = FastestDet(drawOutput=True)
ser_32 = serial.Serial('/dev/ttyAMA0', 115200)
ser_BT = serial.Serial('/dev/ttyAMA3', 9600)

# 定义广义常亮
left='L+30'
right='R+30'
back='L+30'
go='V+30'
stop='V+00'
light_on='OKOK'
light_off='OFFF'
left_time=0.64
right_time=0.72
back_time=1.3
# stop_time表示缓冲时间
stop_time=0.5

# 获取照片数字
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
def get_32():
    hex_data = ser_32.read(1)
    return hex_data

# 这一步用于识别红色条纹的位置，计算误差后传给stm32
def get_road_error(road):
    d=np.argwhere(road[[450,350,250],:] > 250)
    dis=np.mean(d,axis=0)-320
    k=dis[1]
    k=k
    error='E+00'
    k=min(max(-100,k),100)
    x=0.8
    if k>=0:
        error='E'+'-'+str(int(int(k**(x))%100/10))+str(int(k**(x))%10)
    if k<0:
        error='E'+'+'+str(int(int((-k)**(x))%100/10))+str(int((-k)**(x))%10)
    return error,k

# 在路口前(识别数字时)发2，过路口后发(要转弯时)3，到路的尽头后发4，正常发1
def get_cross(road, aim_cross):
    if aim_cross==2:
        if np.sum(road[10:20,:] > 250)>600:
            return 2
        else:
            return 1
    if aim_cross==3:
        if np.sum(road[420:430,:] > 250)>600:
            return 3
        else:
            return 1
    if aim_cross==4:
        if np.sum(road[420:430,:] > 250)<10:
            return 4
        else:
            return 1
    else:
        return 1

# 监测压力传感器监测药物状态，有药为1，无药为0
def get_drug():
    hex_data =  int(ser_32.read(1))
    return hex_data

# 对图片进行分割操作
def get_road(cap):
    _,image=cap.read()
    road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
    # 对图片进行二值化，阈值为30
    _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
    return road

# 直行并检测前方状态
def go_straight(road):
    error,k=get_road_error(road)
    send_order(error,ser_32)

# 立即停止
def car_stop():
    time.sleep(0.5)
    send_order('V+00',ser_32)
    time.sleep(0.2)    
    send_order('V+00',ser_32)
    time.sleep(0.2)
    send_order('V+00',ser_32)
    time.sleep(0.5)

# 转弯
def car_turn(direction):
    time.sleep(0.5)
    if direction=='l':
        # send_order(left,ser_32)
        # time.sleep(left_time)
        # send_order('V+00',ser_32)
        # time.sleep(stop_time)
        send_order('L+90',ser_32)
        time.sleep(2)
    if direction=='r':
        # send_order(right,ser_32)
        # time.sleep(right_time)
        # send_order('V+00',ser_32)
        # time.sleep(stop_time)   
        send_order('R+90',ser_32)
        time.sleep(2) 

# 掉头
def car_back():
    # send_order(back,ser_32)
    # time.sleep(back_time)
    # send_order('V+00',ser_32)
    # send_order('RSET', ser_32) 
    # # 重启串口
    # ser_32.close()
    # ser_32 = serial.Serial('/dev/ttyAMA0', 115200)
    # time.sleep(stop_time)
    time.sleep(1)
    send_order('L+90',ser_32)
    time.sleep(2.5)
    send_order('L+90',ser_32)
    time.sleep(2.5)

# 获取蓝牙信号
def get_BT():
    hex_data = ser_BT.read(1)
    return hex_data

def car_reset():
    time.sleep(1)
    send_order('RSET', ser_32)
    time.sleep(3)

def cap_refresh():
    pass
    # for i in range(15):
    #     #time.sleep(2)
    #     _,image=cap.read()
    #     road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
    #     # 对图片进行二值化，阈值为30
    #     _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
    #     print('first'+str(i))
    #     get_road_error(road)

# 车前进
def car_go():
    time.sleep(0.5)
    send_order(go,ser_32)
    for i in range(10):
        road=get_road(cap)
        go_straight(road)

time.sleep(1)
send_order('901S', ser_32)
time.sleep(3)
#send_order('RSET', ser_32)
#time.sleep(5)
# 确保位置没问题
send_order('L+00', ser_32)

_,image=cap.read()
# cv2.imwrite('num2.jpg',image)
_,given_num=get_photo_number(image)
given_num=given_num[0]
time.sleep(0.9)
send_order('V+00', ser_32)

print('given_number='+str(given_num))
print('begin')
time.sleep(1)
send_order('OKOK',ser_32)
time.sleep(1)
send_order('OFFF',ser_32)
time.sleep(1)

# 等待药物安放
# '''
drug=0
while drug!=b'y':
    drug=get_32()
    time.sleep(0.1)
print('drug is OK')
time.sleep(1)
# '''

# 等待1车就位
cart2_direct=0
while cart2_direct not in [b'1', b'2']:
    cart2_direct = get_BT()
    time.sleep(0.1)

if cart2_direct != b'1':
    cross_go='r'
    print('cart2_direct: r')
else:
    cross_go='l'
    print('cart2_direct: l')

# 现在1车已经就位，我们开始前进

car_go()
for i in range(80):
    road=get_road(cap)
    go_straight(road)
# 判断前面是否是交叉路口
while get_cross(road, 2)!=2:
# while True:
    road=get_road(cap)
    go_straight(road)

time.sleep(1)
car_stop()

print('cross_go: '+cross_go)
# 这个时候我们转弯进入对应的分支
car_turn(cross_go)

car_go()
road=get_road(cap)
for i in range(5):
    road=get_road(cap)
    go_straight(road)
print('go to the branch, 开始识别')

while get_cross(road, 4)!=4:
    road=get_road(cap)
    go_straight(road)
car_stop()

# 此时已经到达终点
send_order(light_on,ser_32)

# 给1车发信号
send_order('G',ser_BT)
time.sleep(1)
send_order('G',ser_BT)
time.sleep(1)
send_order('G',ser_BT)

############################################行程过半###########################################

# 等待1车信号，然后我们掉头回去

while get_BT()!=b'Z':
    time.sleep(0.1)

#当1车就位过后，我们掉头回去
send_order(light_off, ser_32)
car_back()

# 重启摄像头
#del cap
#gc.collect()
#cap=cv2.VideoCapture(0)
#cap_refresh()

print('开始掉头回去了')
car_go()
road=get_road(cap)
while get_cross(road, 4)!=4:
    road=get_road(cap)
    go_straight(road)

car_stop()
send_order(light_on,ser_32)
time.sleep(3)
send_order(light_off,ser_32)

# 此时已就位