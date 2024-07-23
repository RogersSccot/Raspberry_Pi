import cv2
import numpy as np
import serial
import cv2
import numpy as np
import time
from Vision_Net import FastestDet

# 打开摄像机和端口，不轻易运行
ser = serial.Serial('/dev/ttyAMA0', 115200)
ser_BT = serial.Serial('/dev/ttyAMA3', 38400)
cap=cv2.VideoCapture(0)
loo_global=np.zeros((640,480,3),dtype=np.uint8)
deep = FastestDet(drawOutput=True)

# 定义广义常亮
left='L+30'
right='R+30'
back='L+30'
go='V+45'
stop='V+00'
light_on='OKOK'
light_off='OFFF'
left_time=0.64
right_time=0.72
back_time=1.3
# stop_time表示缓冲时间
stop_time=2

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
def get_cross(road, aim_cross):
    if aim_cross==2:
        if np.sum(road[20:30,:] > 250):
            return 2
        else:
            return 1
    if aim_cross==3:
        if np.sum(road[420:430,:] > 250):
            return 3
        else:
            return 1
    if aim_cross==4:
        if np.sum(road[20:30,:] < 20):
            return 4
        else:
            return 1
    else:
        return 1

# 监测压力传感器监测药物状态，有药为1，无药为0
def get_drug(ser):
    hex_data =  int(ser.read(1))
    return hex_data

# 对图片进行分割操作
def get_road(cap):
    _,image=cap.read()
    road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
    # 对图片进行二值化，阈值为30
    _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
    return road

# 直行并检测前方状态
def go_straight(ser, road):
    error,k=get_road_error(road)
    send_order(error,ser)    

# 立即停止
def car_stop(ser):
    send_order('V+00',ser)
    send_order('V+00',ser)
    send_order('V+00',ser)
    time.sleep(stop_time)

# 转弯
def car_turn(ser, direction):
    if direction=='l':
        send_order(left,ser)
        time.sleep(left_time)
        send_order('V+00',ser)
        time.sleep(stop_time)
    if direction=='r':
        send_order(right,ser)
        time.sleep(right_time)
        send_order('V+00',ser)
        time.sleep(stop_time)          

# 掉头
def car_back(ser):
    send_order(back,ser)
    time.sleep(back_time)
    send_order('V+00',ser)
    send_order('RSET', ser) 
    # 重启串口
    ser.close()
    ser = serial.Serial('/dev/ttyAMA0', 115200)
    time.sleep(stop_time)

# 车前进
def car_go(ser):
    send_order(go,ser)
    send_order(go,ser)
    send_order(go,ser)    

send_order('RSET', ser)
# 初始化摄像头，初始化图像
for i in range(30):
    #time.sleep(2)
    _,image=cap.read()
    road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
    # 对图片进行二值化，阈值为30
    _, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
    print('first'+str(i))
    get_road_error(road)

# 首先识别数字
# '''
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
pass

if given_num in [1,2]:
    #----------------------------------------------------------------当数字为1的时候------------------------------------------------------
    # 选择路径
    if given_num==1:
        cross_go='l'
        cross_back='r'
    else:
        cross_go='r'
        cross_back='l'
    car_go(ser)
    road=get_road(cap)
    # 当还没有经过路口时，继续直行
    while get_cross(road, 3)!=3:
        road=get_road(cap)
        go_straight(ser, road)
    # 到达路口，先停下
    car_stop(ser)
    car_turn(ser, cross_go)
    car_go(ser)
    # 当还没有经过路口时，继续直行
    while get_cross(road, 4)!=4:
        road=get_road(cap)
        go_straight(ser, road)
    # 到达路的尽头，先停顿一下，然后点灯
    car_stop(ser)
    send_order(light_on,ser)
    time.sleep(3)
    # 监测药的状态
    # drug='y'
    # while drug!='n':
    #     drug=get_drug()
    # 当药物没有时
    send_order(light_off,ser)
    # '''
    # 现在开始掉头回去------------------------------------------------------行程过半--------------------------------------------------------
    car_back(ser)
    # 开始走
    car_go(ser)
    road=get_road(cap)
    while get_cross(road, 3)!=3:
        road=get_road(cap)
        go_straight(ser, road)
    car_stop(ser)
    car_turn(ser, cross_back)
    car_go(ser)
    road=get_road(cap)
    # 当还没有经过路口时，继续直行
    while get_cross(road, 4)!=4:
        road=get_road(cap)
        go_straight(ser, road)
    # 到达路的尽头，先停顿一下
    car_stop(ser)
    send_order(light_off, ser)
    # '''

if given_num in [3,4,5,6,7,8]:
    car_go(ser)
    road=get_road(cap)
    # 先走一段路
    for i in range(50):
        go_straight(ser, road)
        road=get_road(cap)
    # 判断前面是否是交叉路口
    while get_cross(ser, 2)!=2:
        go_straight(ser, road)
        road=get_road(cap)      
    car_stop(ser)
    
    # 先识别这里的数字 
    _,image=cap.read()
    cross_num_position,cross_num=get_photo_number(image)

    # 先让小车往前面走一点
    car_go(ser)
    cv2.imwrite('cross1.jpg',image)
    print(cross_num,cross_num_position)
    # 如果这里有我们想要的数字，就是中部药房
    if given_num in cross_num:
        aim_index=cross_num.index(given_num)
        aim_position=cross_num_position[aim_index][0]
        if aim_position<320:
            cross_go='l'
            cross_back='r'
        else:
            cross_go='r'
            cross_back='l'
        time.sleep(0.9)
        car_stop(ser)
        car_turn(ser, cross_go)
        # 此时我们已经转过弯了
        car_go(ser)
        while get_cross(road, 4)!=4:
            go_straight(ser, road)
            road=get_road(cap) 
        car_stop(ser)
        # 此时我们已经到达终点
        send_order(light_on,ser) 
        # 送到药的状态
        time.sleep(3)
        # 监测药的状态
        # drug='y'
        # while drug!='n':
        #     drug=get_drug()
        # 当药物没有时
        send_order(light_off,ser)   

        # 现在开始掉头回去------------------------------------------------------行程过半--------------------------------------------------------
        car_back(ser)
        car_go(ser)
        road=get_road(cap)
        while get_cross(ser, 3)!=3:
            go_straight(ser, road)
            road=get_road(cap) 
        car_stop(ser)
        # 十字路口转弯
        car_turn(ser, cross_back)
        # 继续直走
        car_go(ser)
        road=get_road(cap)
        while get_cross(ser, 4)!=4:
            go_straight(ser, road)
            road=get_road(cap)
        car_stop(ser)
        send_order(light_on,ser)
        time.sleep(stop_time)
        send_order(light_off,ser)                     
    else:
        car_go(ser)
        road=get_road(ser)
        for i in range(10):
            go_straight(ser, road)
            road=get_road(cap)
        # 判断前面是否是交叉路口
        while get_cross(ser, 2)!=2:
            go_straight(ser, road)
            road=get_road(cap)      
        car_stop(ser)
        # 现在已经到达了中间的十字路口
        _,image=cap.read()
        cross_num_position,cross_num=get_photo_number(image)
        car_go(ser)
        cv2.imwrite('cross2.jpg',image)        
        aim_index=cross_num.index(given_num)
        aim_position=cross_num_position[aim_index][0]
        if aim_position<320:
            cross1_go='l'
            cross1_back='r'
        else:
            cross1_go='r'
            cross1_back='l'
        time.sleep(0.9)
        car_stop(ser)
        # 现在可以开始转弯
        car_turn(ser, cross1_go)
        # 继续直行
        car_go(ser)
        while get_cross(road, 2)!=2:
            go_straight(ser, road)
            road=get_road(cap) 
        car_stop(ser) 
        # 再次识别数字
        # 现在已经到达了边缘的十字路口
        _,image=cap.read()
        cross_num_position,cross_num=get_photo_number(image)
        car_go(ser)
        cv2.imwrite('cross3.jpg',image)        
        aim_index=cross_num.index(given_num)
        aim_position=cross_num_position[aim_index][0]
        if aim_position<320:
            cross2_go='l'
            cross2_back='r'
        else:
            cross2_go='r'
            cross2_back='l'
        time.sleep(0.9)
        car_stop(ser)
        # 现在可以开始转弯
        car_turn(ser, cross2_go)
        # 直行到尽头
        car_go(ser)
        # while get_cross(road, 2)!=2:
        for i in range(40):
            go_straight(ser, road)
            road=get_road(cap) 
        car_stop(ser) 

        send_order(light_on,ser)
        time.sleep(3)
        # 监测药的状态
        # drug='y'
        # while drug!='n':
        #     drug=get_drug()
        # 当药物没有时
        send_order(light_off,ser)
        car_back(ser)
        # 回去的第一段路
        for i in range(8):
            go_straight(ser, road)
            road=get_road(cap)
        # 判断前面是否是交叉路口
        while get_cross(ser, 3)!=3:
            go_straight(ser, road)
            road=get_road(cap)      
        car_stop(ser)        
        # 边缘路口转弯
        car_turn(ser, cross2_back)
        # 继续直走
        car_go(ser)
        road=get_road(cap)
        while get_cross(ser, 3)!=3:
            go_straight(ser, road)
            road=get_road(cap)
        car_stop(ser)                
        car_turn(ser, cross1_back)
        # 继续直走
        car_go(ser)
        for i in range(120):
            go_straight(ser, road)
            road=get_road(cap)
        # 判断前面是否是交叉路口
        while get_cross(ser, 4)!=4:
            go_straight(ser, road)
            road=get_road(cap)      
        car_stop(ser)
        send_order(light_on,ser)
        time.sleep(stop_time)
        send_order(light_off,ser)    





