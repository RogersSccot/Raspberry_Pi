#%%
'''
1.首先获取图片识别，在识别数字之后开始控制电机的运动
2.在运动的过程中，将误差传给stm32，同时记住当时的路径操作
3.同时，在运动结束后，识别图像，然后将下一次的指令发给stm32
4.注意路径的格式（只有有限种情况可能发生）
5.到达终点后亮红灯
'''
import cv2
import numpy as np
import numpy
import serial
import time
#%%
# 打开摄像机和端口，不轻易运行
ser = serial.Serial('/dev/ttyAMA0', 115200)
capture=cv2.VideoCapture(0)

#%%
'''这一步用于识别目标视野里面的数字图片，识别成功就是1~8，返回识别到的数字以及其位置，否则就是0'''
'''方案一：直接使用模式匹配实现'''
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
#%%
'''此时已经得到aim_number，首先先尝试抵达终点'''
aim_number=3
sleep_time=0
for i in range(50):
    get_send_road_error(capture)
print('begin')
while True:
    #road_go=['G+05']*10+['L+05']*2+['G+05']*5
    #road_go=['G+08']*100
    #for a in road_go:
    a='G+05'
    print(a)
    send_order(a)
    #首先走完一小段
    b=0
    f=get_send_road_error(capture)
    #用于判断是否走完这道命令
    while b==0:
        #发送误差使得不断修正偏差
        # get_send_road_error(capture)
        b=get_stm32feed(ser)
    time.sleep(sleep_time)
    
    send_order(f)
    print(f)
    b=0
    while b==0:
        #发送误差使得不断修正偏差
        # get_send_road_error(capture)
        b=get_stm32feed(ser)
    #time.sleep(sleep_time)
    # 当循环结束证明已经达到节点，这时候直接执行后面的命令
    
    #当走完时会点灯
    # send_order('OKOK')
time.sleep(1)

'''
    road_turn=['L+10']*2
    for a in road_turn:
        send_order(a)
        #首先走完一小段
        b=0
        #用于判断是否走完这道命令
        while b==0:
            #发送误差使得不断修正偏差
            # get_send_road_error(capture)
            b=get_stm32feed(ser)
        #send_order(get_send_road_error(capture))
        time.sleep(sleep_time)
    '''
'''
    road_back=get_backroad(road_go)
    
    #检测药物是否被拿走
    # while drug:
    #     drug=get_drug(ser)
    #当药拿走后，开始返回
    time.sleep(sleep_time)
    time.sleep(1)
    for a in road_back:
        send_order(a)
        print(a)
        #首先走完一小段
        b=0
        f=get_send_road_error(capture)
        #用于判断是否走完这道命令
        while b==0:
            #发送误差使得不断修正偏差
            # get_send_road_error(capture)
            b=get_stm32feed(ser)
        time.sleep(sleep_time)
        
        send_order(f)
        print(f)
        b=0
        while b==0:
            #发送误差使得不断修正偏差
            # get_send_road_error(capture)
            b=get_stm32feed(ser)
        time.sleep(sleep_time)
        '''
#%%

