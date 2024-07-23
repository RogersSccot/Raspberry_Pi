#%%
import cv2
import numpy as np
import matplotlib.pyplot as plt
import serial
#%%
ser = serial.Serial('/dev/ttyAMA0', 115200)
capture=cv2.VideoCapture(0)
#%%
def get_photo_number(capture):
    num=[]
    num_position=[]
    _,image=capture.read()
    image_gray=255-cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, image_gray = cv2.threshold(image_gray, 160, 255, cv2.THRESH_BINARY)
    image_gray=image_gray.astype(np.float32)-127
    for i in range(len(num_img)):
        kernel=num_img[i]/50
        try:
            dst2=cv2.filter2D(image_gray/50,cv2.CV_32F,kernel)
            num_center = np.argwhere((dst2 >= np.max(dst2)) & (dst2 >= 25000))
            # draw_number(image, num_center[0], i)
            if num_center==np.array([[]]):
                num.append(i+1)
                num_position.append(num_center[0])
        except cv2.error:
            num_center=[0,0]
        except IndexError:
            num_center=[0,0]
    #这一步识别图片，成功后返回对应的数据
    return num,num_position

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
    k=-k/100
    if k>=0:
        error='E'+'+'+str(int(int(k*10)%100/10))+str(int(k*10)%10)
    if k<0:
        error='E'+'-'+str(int(int((-k)*10)%100/10))+str(int((-k)*10)%10)
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

def get_cross(capture):
    _,image=capture.read()
    road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,1].astype(np.float32)+image[:,:,0].astype(np.float32)))
    # 对图片进行二值化，阈值为30
    ret, road = cv2.threshold(road_red, 40, 255, cv2.THRESH_BINARY)
    # 这是每一行中大于250的点的个数
    sides=np.zeros((30,501))
    sides[0:30,0:250]=road[410:440,0:250]
    sides[0:30,251:501]=road[410:440,389:639]
    above=np.zeros((110,640))
    above[0:110,0:640]=road[360:470,0:640]
    sides_250 = np.sum(sides > 250)
    above_250 = np.sum(above > 250)
    print(sides_250)
    if sides_250>7000:
        return 2
    if above_250<100:
        return 0
    else:
        return 1

#%%
while True:
    _,image=capture.read()
    road_red=np.abs(image[:,:,2].astype(np.float32)-0.5*(image[:,:,1].astype(np.float32)+image[:,:,0].astype(np.float32)))
    # 对图片进行二值化，阈值为30
    ret, road = cv2.threshold(road_red, 40, 255, cv2.THRESH_BINARY)
    cv2.imshow('image', image)
    cv2.imshow('road', road)
    cv2.waitKey(1)
#%%
#cv2.imwrite('image.png',image)