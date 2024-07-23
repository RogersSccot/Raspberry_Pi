#%%
import cv2
import numpy as np
import numpy
import serial
import time
import cv2
import threading
#from Vision_Net import FastestDet
#%%
cap=cv2.VideoCapture(0)
#%%
'''图片识别，识别是多少就是多少，没有识别到就返回零'''
'''
loo_global=np.zeros((640,480,3),dtype=np.uint8)
def updatephoto():
    global loo_global
    while True:
        map,loo_global=cap.read()
t=threading.Thread(target=updatephoto)
t.start()
lock=threading.Lock()

def photo_processing():
    #读取图像
    lock.acquire()
    frame=loo_global
    lock.release()
    #roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]   #对原始图像进行裁切
    return frame

photo = photo_processing()
deep = FastestDet(drawOutput=True)
connected = cap.isOpened()
#'''
'''
def get_photo_number(cap):
    _, frame = cap.read()
    out = deep.detect(frame)
    num_position=[]
    num=[]
    for i in range(len(out)):
        num_position.append(out[i][0])
        num.append(out[i][1])
    return  num_position,num
'''
'''在判断好数字后将决定前进的方向，输入为G±nn，L±nn，R±nn，到达终点后，发送OKOK'''
def send_order(order):
    encoded_data = order.encode()
    ser.write(encoded_data)

'''获得stm32的反馈，当stm32检测到已到达一个cornor时，或者一段命令已经执行完后，发送1给树莓派'''
def get_stm32feed(ser):
    # hex_data =  int(ser.read(1).hex().upper(), 16)
    hex_data =  int(ser.read(1))
    return hex_data

'''这一步用于识别红色条纹的位置，计算误差后传给stm32'''
'''
def get_send_road_error(cap):
    _,image=cap.read()
    road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
    # 对图片进行二值化，阈值为30
    ret, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
    # 这是每一行中大于250的点的个数
    mark1=np.sum(road[409:480,:] > 250, axis=1)>15
    # 计算满足条件的行的平均值点
    d1=np.argwhere(road[409:480,:][mark1] > 250)
    e1=np.array([d1[:,0]+409,d1[:,1]])
    average_points1=np.mean(e1,axis=1)
    mark2=np.sum(road[338:408,:] > 250, axis=1)>15
    # 计算满足条件的行的平均值点
    d2=np.argwhere(road[338:408,:][mark2] > 250)
    e2=np.array([d2[:,0]+338,d2[:,1]])
    average_points2=np.mean(e2,axis=1)
    # k=(average_points1[1]-average_points2[1])/(average_points1[0]-average_points2[0])
    error='E+00'
    k=average_points2[1]-320
    k=-k/100
    if k>=0:
        error='E'+'+'+str(int(int(k*10)%100/10))+str(int(k*10)%10)
    if k<0:
        error='E'+'-'+str(int(int((-k)*10)%100/10))+str(int((-k)*10)%10)
    return error
'''

def get_send_road_error(capture):
    _,image=capture.read()
    road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,0].astype(np.float32)+image[:,:,1].astype(np.float32)))
    # 对图片进行二值化，阈值为30
    ret, road = cv2.threshold(road_red, 45, 255, cv2.THRESH_BINARY)
    # 这是每一行中大于250的点的个数
    mark1=np.sum(road[409:480,:] > 250, axis=1)>15
    # 计算满足条件的行的平均值点
    d1=np.argwhere(road[409:480,:][mark1] > 250)
    e1=np.array([d1[:,0]+409,d1[:,1]])
    average_points1=np.mean(e1,axis=1)
    mark2=np.sum(road[338:408,:] > 250, axis=1)>15
    # 计算满足条件的行的平均值点
    d2=np.argwhere(road[338:408,:][mark2] > 250)
    e2=np.array([d2[:,0]+338,d2[:,1]])
    average_points2=np.mean(e2,axis=1)
    k=(average_points1[1]-average_points2[1])/(average_points1[0]-average_points2[0])
    error='E+00'
    k=average_points2[1]-320
    k=-k/10
    if k>=0:
        error='E'+'+'+str(int(int(k*10)%100/10))+str(int(k*10)%10)
    if k<0:
        error='E'+'-'+str(int(int((-k)*10)%100/10))+str(int((-k)*10)%10)
    print(error)
    return error


'''这一步用于当到达终点后，立刻获取返回的路径'''
def get_backroad(road_go):
    print('get_back')
    road_back=[]
    for a in [-i for i in range(1, len(road_go)+1)]:
        c=road_go[a]
        if c[0]=='G':
            road_back.append(c)
        if c[0]=='L':
            road_back.append('R'+c[1:4])
        if c[0]=='R':
            road_back.append('L'+c[1:4])
    return road_back

'''监测压力传感器监测药物状态，有药为1，无药为0'''
def get_drug(ser):
    hex_data =  int(ser.read(1))
    return hex_data

'''获取路面状况，正常发1，遇见十字路口前面发2，过十字路口发3，遇见道路尽头发4'''
def get_cross(cap):
    _,image=cap.read()
    road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,1].astype(np.float32)+image[:,:,0].astype(np.float32)))
    # 对图片进行二值化，阈值为30
    ret, road = cv2.threshold(road_red, 40, 255, cv2.THRESH_BINARY)
    # 这是每一行中大于250的点的个数
    sides=np.zeros((30,501))
    sides[0:30,0:250]=road[410:440,0:250]
    sides[0:30,251:501]=road[410:440,389:639]
    sides_front=np.zeros((30,501))
    sides_front[0:30,0:250]=road[60:90,0:250]
    sides_front[0:30,251:501]=road[60:90,389:639]
    above=np.zeros((110,640))
    above[0:110,0:640]=road[360:470,0:640]
    sides_250 = np.sum(sides > 250)
    sides_front_250 = np.sum(sides_front > 250)
    above_250 = np.sum(above > 250)
    print(sides_250)
    if sides_front_250>2000:
        return 2
    if sides_250>2000:
        return 3
    if above_250<100:
        return 4
    else:
        return 1

'''go straight with red guide'''
def go_straight(cap):
    a='G+05'
    road_go=['G+08']
    for a in road_go:
        print(a)
        send_order(a)
        #首先走完一小段
        b=0
        f=get_send_road_error(cap)
        #用于判断是否走完这道命令
        while b==0:
            #发送误差使得不断修正偏差
            # get_send_road_error(capture)
            b=get_stm32feed(ser)
        #time.sleep(sleep_time)
        
        send_order(f)
        print(f)
        b=0
        while b==0:
            #发送误差使得不断修正偏差
            # get_send_road_error(capture)
            b=get_stm32feed(ser)

sleep_time=3
def go_and_stop(cap,ser,stop_con):
    ser.close()
    time.sleep(sleep_time)
    ser = serial.Serial('/dev/ttyAMA0', 115200)
    while  get_cross(cap)!=stop_con:
        go_straight(cap,ser)
#%%
ser = serial.Serial('/dev/ttyAMA0', 115200)
#%%
'''
工作过程：参赛者手动将小车摆放在药房处（车头投影在门口区域内，面向
病房），手持数字标号纸张由小车识别病房号，将约 200g 药品一次性装载到送
药小车上；小车检测到药品装载完成后自动开始运送；小车根据走廊上的标识信
息自动识别、寻径将药品送到指定病房（车头投影在门口区域内），点亮红色指
示灯，等待卸载药品；病房处人工卸载药品后，小车自动熄灭红色指示灯，开始
返回；小车自动返回到药房（车头投影在门口区域内，面向药房）后，点亮绿色
指示灯
'''
# 1.first we catch the given photo number, then we light
# 2.after that, we detect the drug condition
for i in range(30):
    #time.sleep(2)
    print('first'+str(i))
    get_send_road_error(cap)
'''
_,given_number1=get_photo_number(cap)
given_number=int(given_number1[0])
print('given_number:',given_number)
time.sleep(3)

'''
#send_order('OKOK')
'''
drug=0
#while drug==0:
#   drug=get_drug(ser)
# 3.if the drug condition is 1, we light the red light, and start
'''
given_number=1
if given_number in [1,2]:
    while get_cross(cap)!=3:
        a='G+10'
        print(a)
        send_order(a)
        #首先走完一小段
        b=0
        f=get_send_road_error(cap)
        #用于判断是否走完这道命令
        while b==0:
            #发送误差使得不断修正偏差
            # get_send_road_error(capture)
            b=get_stm32feed(ser)
            
        send_order(f)
        print(f)
        b=0
        while b==0:
            #发送误差使得不断修正偏差
            # get_send_road_error(capture)
            b=get_stm32feed(ser)
        #time.sleep(sleep_time)
    #send_order('G+05')
    time.sleep(1)
    if given_number==1:
        a1='R+45'
        send_order('L+45')
    else:
        a1='L+20'
        send_order('R+20')
    # now we get across the cross
    #go_and_stop(cap,ser,4)
    ser.close()
    time.sleep(sleep_time)
    ser = serial.Serial('/dev/ttyAMA0', 115200)
    while  get_cross(cap)!=4:
        a='G+10'
        print(a)
        send_order(a)
        #首先走完一小段
        b=0
        f=get_send_road_error(cap)
        #用于判断是否走完这道命令
        while b==0:
            #发送误差使得不断修正偏差
            # get_send_road_error(capture)
            b=get_stm32feed(ser)
            
        send_order(f)
        print(f)
        b=0
        while b==0:
            #发送误差使得不断修正偏差
            # get_send_road_error(capture)
            b=get_stm32feed(ser)
        #time.sleep(sleep_time)
    #send_order('G+03')
    # now we achieve the end
    #send_order('OKOK')
    #while drug==1:
    #    drug=get_drug(ser)
    # now we start to go back,first we turn back
    
    send_order('B+30')
    ser.close()
    time.sleep(sleep_time)
    ser = serial.Serial('/dev/ttyAMA0', 115200)
    while  get_cross(cap)!=3:
        a='G+10'
        print(a)
        send_order(a)
        #首先走完一小段
        b=0
        f=get_send_road_error(cap)
        #用于判断是否走完这道命令
        while b==0:
            #发送误差使得不断修正偏差
            # get_send_road_error(capture)
            b=get_stm32feed(ser)
            
        send_order(f)
        print(f)
        b=0
        while b==0:
            #发送误差使得不断修正偏差
            # get_send_road_error(capture)
            b=get_stm32feed(ser)
        #time.sleep(sleep_time)
    #send_order('G+03')
    time.sleep(1)
    # now we turn
    send_order(a1)
    # go straight till we torch the end
    ser.close()
    time.sleep(sleep_time)
    ser = serial.Serial('/dev/ttyAMA0', 115200)
    while  get_cross(cap)!=4:
        a='G+10'
        print(a)
        send_order(a)
        #首先走完一小段
        b=0
        f=get_send_road_error(cap)
        #用于判断是否走完这道命令
        while b==0:
            #发送误差使得不断修正偏差
            # get_send_road_error(capture)
            b=get_stm32feed(ser)
            
        send_order(f)
        print(f)
        b=0
        while b==0:
            #发送误差使得不断修正偏差
            # get_send_road_error(capture)
            b=get_stm32feed(ser)
        #time.sleep(sleep_time)
    #send_order('G+03')
