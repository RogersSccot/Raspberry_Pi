import cv2
import numpy as np
import serial
import time

# 打开摄像机和端口，不轻易运行
# capture=cv2.VideoCapture(0)
# ser.close()
# ser = serial.Serial('COM10', 115200)
ser = serial.Serial('/dev/ttyAMA0', 115200)

'''这一步用于识别目标视野里面的数字图片，识别成功就是1~8，返回识别到的数字以及其位置，否则就是0'''
'''方案一：直接使用模式匹配实现'''


'''在判断好数字后将决定前进的方向，输入为G±nn，L±nn，R±nn，到达终点后，发送OKOK'''
def send_order(order):
    encoded_data = order.encode()
    ser.write(encoded_data)

'''获得stm32的反馈，当一段命令已经执行完后，发送1给树莓派'''
def get_stm32feed(ser):
    # hex_data =  int(ser.read(1).hex().upper(), 16)
    hex_data=int(ser.read(1))
    return hex_data

'''这一步用于识别红色条纹的位置，计算误差后传给stm32'''
def get_send_road_error(capture):
    _,image=capture.read()
    road_red=np.abs(image[:,:,2]-0.5*(image[:,:,1]+image[:,:,0]))
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
    if k>=0:
        error='E'+'+'+str(int(int(k*10)%100/10))+str(int(k*10)%10)
    if k<0:
        error='E'+'-'+str(int(int((-k)*10)%100/10))+str(int((-k)*10)%10)
    print(error)

'''这一步用于当到达终点后，立刻获取返回的路径'''
def get_backroad(road_go):
    print('get_back')
    road_back=[]
    for a in [-i for i in range(1, len(road_go)+1)]:
        c=road_go[a][0]+'-'+road_go[a][2:4]
        road_back.append(c)
    return road_back

'''监测压力传感器监测药物状态，有药为1，无药为0'''
def get_drug(ser):
    hex_data =  int(ser.read(1))
    return hex_data

def get_corner(capture):
    _,image=capture.read()
    road_red=np.abs(image[:,:,2]-0.5*(image[:,:,1]+image[:,:,0]))
    # 对图片进行二值化，阈值为30
    ret, road = cv2.threshold(road_red, 50, 255, cv2.THRESH_BINARY)
    # 这是每一行中大于250的点的个数
    sides=np.zeros((30,501))
    sides[0:30,0:250]=road[110:140,0:250]
    sides[0:30,251:501]=road[110:140,389:639]
    sides_250 = np.sum(sides > 250)
    above=np.zeros((110,640))
    above[0:110,0:640]=road[0:110,0:640]
    above_250 = np.sum(above > 250)
    if (sides_250>30000)|(above_250<0):
        return 1
    else:
        return 0
    
send_order('R+60')
aim_number=1
'''此时已经得到aim_number，首先先尝试抵达终点'''
#send_order("L+28")
send_order("R+28")
'''
road_go=['G+10']*10
while True:
    send_order('G+10')
    #首先走完一小段
    b=0
    #用于判断是否走完这道命令
    while b==0:
        #发送误差使得不断修正偏差
        # get_send_road_error(capture)
        b=get_stm32feed(ser)
    #time.sleep(0.5)
#'''

# send_order('B+05')


'''
    # get_photo_number(capture)
    send_order('L+45')
    while get_corner(capture)==0:
        get_send_road_error(capture)
    # get_drug(ser)
    send_order('B+90')
    while get_corner(capture)==0:
        get_send_road_error(capture)
    # get_photo_number(capture)
    send_order('L+45')
    while get_corner(capture)==0:
        get_send_road_error(capture)
'''







