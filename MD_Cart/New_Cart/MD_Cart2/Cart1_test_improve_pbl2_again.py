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
    send_order('V+00',ser_32)
    time.sleep(0.2)    
    send_order('V+00',ser_32)
    time.sleep(0.2)
    send_order('V+00',ser_32)
    time.sleep(stop_time)

# 转弯
def car_turn(direction):
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
    send_order('L+90',ser_32)
    time.sleep(2.5)
    send_order('L+90',ser_32)
    time.sleep(2.5)

# 获取蓝牙信号
def get_BT():
    hex_data = ser_BT.read(1)
    return hex_data

def car_reset():
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
    send_order(go,ser_32)
    for i in range(5):
        #go_straight(road)
        road=get_road(cap)
    for i in range(10):
        road=get_road(cap)
        go_straight(road)

send_order('901S', ser_32)
time.sleep(3)
#send_order('RSET', ser_32)
#time.sleep(5)
send_order('L+00', ser_32)

_,image=cap.read()
# cv2.imwrite('num2.jpg',image)
_,given_num=get_photo_number(image)
given_num=given_num[0]
time.sleep(0.5)
send_order('V+00', ser_32)

print('given_number='+str(given_num))
print('begin')
time.sleep(1)
send_order('OKOK',ser_32)
time.sleep(1)
send_order('OFFF',ser_32)
time.sleep(1)

# 等待药物安放
drug=0
while drug!=b'y':
    drug=get_32()
    time.sleep(0.1)
print('drug is OK')
time.sleep(1)

#在完成识别数字过后，我们需要根据数字来判断下一步的动作
#首先我们前进到第二个路口

#del cap
#gc.collect()
#cap=cv2.VideoCapture(0)

car_go()
for i in range(80):
    road=get_road(cap)
    go_straight(road)
# 判断前面是否是交叉路口
while get_cross(road, 2)!=2:
# while True:
    road=get_road(cap)
    go_straight(road)

# 这个时候已经可以识别数字了
print('到达第3个路口，正在读取第3个路口的图像')
_,image=cap.read()
cross_num_position,cross_num=get_photo_number(image)
cv2.imwrite('cross1.jpg',image)
time.sleep(0.9)
print(cross_num,cross_num_position)
# 因为是第二问，所以肯定有我们的数字
aim_index=cross_num.index(given_num)
aim_position=cross_num_position[aim_index][0]
if aim_position<320:
    # 在左边
    cross_go='l'
    cross_back='r'
    cart2_direct='1'
else:
    # 在右边
    cross_go='r'
    cross_back='l'
    cart2_direct='2'

car_stop()

print('cross1_go: '+cross_go)
# 这个时候我们转弯进入对应的分支
car_turn(cross_go)

#del cap
#gc.collect()
#cap=cv2.VideoCapture(0)

# 这个时候我们已经到了第二个路口
car_go()
# 判断前面是否是交叉路口
while get_cross(road, 2)!=2:
# while True:
    road=get_road(cap)
    go_straight(road)

# 这个时候已经可以识别数字了
print('到达分支路口')
_,image=cap.read()
cross_num_position,cross_num=get_photo_number(image)
cv2.imwrite('cross2.jpg',image)
time.sleep(0.9)
print(cross_num,cross_num_position)
# 因为是第二问，所以肯定有我们的数字
aim_index=cross_num.index(given_num)
aim_position=cross_num_position[aim_index][0]
if aim_position<320:
    # 在左边
    cross2_go='l'
    cross2_back='r'
    cart2_direct='3'
else:
    # 在右边
    cross2_go='r'
    cross2_back='l'
    cart2_direct='4'

car_stop()

print('cross2_go: '+cross_go)
# 这个时候我们转弯进入对应的分支
car_turn(cross_go)

car_go()
road=get_road(cap)
for i in range(5):
    road=get_road(cap)
    go_straight(road)
print('go to the branch，开始识别')

while get_cross(road, 2)!=2:
    road=get_road(cap)
    go_straight(road)
time.sleep(0.5)
car_stop()

# 此时已就位
send_order(light_on,ser_32)

# 给2车发信号
# send_order(cart2_direct,ser_BT)

#等待2车就位
# cart_position=0
# while cart_position!=b'1':
#     cart2=get_BT()
#     time.sleep(0.1)

# 等待卸载药物
drug=0
while drug!=b'n':
    drug=get_32()
    print(drug)
    time.sleep(0.1)
########################################行程过半###########################################
#当二车就位过后，我们开始返回
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
while get_cross(road, 2)!=2:
    road=get_road(cap)
    go_straight(road)
cv2.imwrite('cross2_back.jpg',road)
time.sleep(0.5)
car_stop()
time.sleep(0.4)
# 分支路口转弯
print('分支路口转弯')
car_turn(cross2_back)

car_go()
road=get_road(cap)
while get_cross(road, 2)!=2:
    road=get_road(cap)
    go_straight(road)
cv2.imwrite('cross1_back.jpg',road)
time.sleep(0.5)
car_stop()
time.sleep(0.4)
# 分支路口转弯
print('分支路口转弯')
car_turn(cross_back)

car_go()
for i in range(80):
    road=get_road(cap)
    go_straight(road)
print('延时完毕，开始监测')
while get_cross(road, 4)!=4:
    go_straight(road)
    road=get_road(cap)
time.sleep(0.5)
car_stop()
send_order(light_on,ser_32)
time.sleep(stop_time)
send_order(light_off,ser_32)

