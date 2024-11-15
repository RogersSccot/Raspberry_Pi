# 导入必要的函数库
import cv2
import numpy as np
import math
import serial
import threading
import time
from Vision_Net import FastestDet

# 打开摄像头，占用内存大，不轻易运行
capture=cv2.VideoCapture(0)
loo_global=np.zeros((640,480,3),dtype=np.uint8)
deep = FastestDet(drawOutput=True)

# 打开串口
ser_32 = serial.Serial('/dev/ttyAMA0', 921600)

PBL=0
aim_point22=17
aim_point23=0
last_car_point='C00'
car_point='C00'
car_center=[0,0]
grid_str=['0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','N']
grid_int=['00','01','02','03','04','05','06','07','08','09','10','11','12','13','14','15','16']
grid_cross=[[1,4],[2,5,0],[3,6,1],[7,2],[5,8,0],[6,9,4,1],[7,10,5,2],[11,6,3],[9,12,4],[10,13,8,5],[11,14,9,6],[15,10,7],[13,8],[14,12,9],[15,13,10],[14,11]]
grid_str_int={'0':0,'1':1,'2':2,'3':3,'4':4,'5':5,'6':6,'7':7,'8':8,'9':9,'A':10,'B':11,'C':12,'D':13,'E':14,'F':15,'N':16}
M=np.load('/home/saluo/Desktop/Tower_Car/M.npy')
# 我们需要将我们读取到的图像映射成一个平整的图像
ad=cv2.imread('/home/saluo/Desktop/Tower_Car/grid.jpg')
ad_grey=cv2.cvtColor(ad, cv2.COLOR_BGR2GRAY)

# 开启单独进程，获取题目信号
'''
通信协议
1.收到Axx，表示2.1问的目标点
2.收到Bxx，表示2.2问的目标点
3.收到Cxx，表示2.3问的目标点
'''
# 获取照片数字 
def get_photo_number(frame):
    out = deep.detect(frame)
    num_position=[]
    num=[]
    for i in range(len(out)):
        num_position.append(out[i][0])
        num.append(str(out[i][1]))
    return  num_position,num

def get_PBL():
    global PBL, aim_point22, aim_point23
    print('读取题目信息')
    while True:
        PBL = ser_32.read(3)
        print('mail ='+str(PBL))
        if PBL[0]==66:
            PBL=PBL.decode('utf-8')
            aim_point22=grid_str_int[PBL[2]]
        elif PBL[0]==67:
            PBL=PBL.decode('utf-8')
            aim_point23=grid_str_int[PBL[2]]

get_PBL_threading = threading.Thread(target=get_PBL)
get_PBL_threading.start()

# 发送命令，通用函数
'''
通信协议
1.发送Lxx，Rxx，Fxx将会直接反馈给小车
2.发送Pxx，等一系列点用于指定路径
3.发送Cxx，将用于指定小车位置
4.发送Mxx，将用于修改地图布局
'''

def send_order(order, ser):
    print('order='+order)
    encoded_order = order.encode()
    ser.write(encoded_order)

# 获取消息，通用函数
def get_mail(ser):
    encoded_mail = ser.read(1)
    print('mail='+encoded_mail)

# 获取并处理图像
def get_image():
    # 开全局变量处理，分理处红绿蓝
    global image, image_red, image_green, image_blue
    # 从摄像头获取图像
    _,image=capture.read()
    image_red=image[:,:,2].astype(np.float32)
    image_green=image[:,:,1].astype(np.float32)
    image_blue=image[:,:,0].astype(np.float32)
    #cv2.imshow('image',image)
    #cv2.waitKey(1)

# 突出图像中的红色目标
def get_red():
    global image, image_red, image_green, image_blue
    image_red_only=image_red*2-image_blue-image_green
    image_red_only[image_red_only<100]=0
    image_red_only[image_red_only>100]=255
    return image_red_only

# 颜色识别找车
def find_car_red(image_gray):
    global car_center
    try:
        d1=np.argwhere(image_gray>100)
        red1=np.mean(d1,axis=0)
        red2=np.array([red1[1],red1[0],1])
        red3=np.dot(M,red2)
        car_center=[red3[1]/red3[2],red3[0]/red3[2]]
    except:
        car_center=car_center
    return car_center

# 定位函数，定位车在哪个格子里
def locate_car(car_center):
    # 首先找到小车在16宫格内的坐标
    car_center_M=car_center
    car_index=int(car_center_M[0]/242)*4+int(car_center_M[1]/242)
    car_point='C'+str(int(car_index/10))+str(car_index%10)
    return car_point,car_index

# 找到小车的点坐标
def get_car_point():
    get_image()
    red_only=get_red()
    car_center=find_car_red(red_only)
    car_point,car_index=locate_car(car_center)
    return car_point,car_index

# 做一个映射变换
def image_trans(image_to_trans):
    image_after_trans = cv2.warpPerspective(image_to_trans,M,(ad.shape[1],ad.shape[0]))
    image_after_trans=cv2.cvtColor(image_after_trans, cv2.COLOR_BGR2GRAY)
    _,image_after_trans=cv2.threshold(image_after_trans, 127, 255, cv2.THRESH_BINARY)
    return image_after_trans

# 获取当前地图的数字
def get_grid_num(image_tra):
    grid_now_str=['N']*16
    grid_now_int=[16]*16
    for i in range(16):
        x=int(i/4)*242
        y=int(i%4)*242
        pi=image_tra[x+20:x+222,y+20:y+222].astype(np.uint8)
        pi3=np.zeros((202,202,3),dtype=np.uint8)
        pi3[:,:,0]=pi
        pi3[:,:,1]=pi
        pi3[:,:,2]=pi
        _,num=get_photo_number(pi3)
        try:
            grid_now_str[i]=num[0]
            grid_now_int[i]=grid_str_int[num[0]]
        except:
            pass
    # print(grid_change_int)
    return grid_now_str, grid_now_int

def find_road(aim_point22, grid_coverd_int):
    road22=[[0]]
    road_final=[]
    while aim_point22 not in road_final:
        try:
            if len(road22)>1000:
                print('no way')
                return 0
            if aim_point22 not in grid_coverd_int:
                print('aim_point22 not in grid_coverd_int')
                return 0
            # 用于记录所有可能路径的最后一位
            road_final=[]
            # i表示保存的一条可能路径
            for i in road22:
                # print('i='+str(i))
                # 找出这条路径的最后一位，找出最后一位的所有可能下一位
                for j in grid_cross[i[-1]]:
                    # print('j='+str(j))
                    k=i.copy()
                    if (j not in k) and (j not in road_final) and (j in grid_coverd_int):
                        k.append(j)
                        road22.append(k)
                        road_final.append(j)
        except:
            print('break')
            return 0
    # print(road22)
    road_possible=[]
    road_length=[]
    for r in road22:
        if r[-1]==aim_point22:
            road_possible.append(r)
            road_length.append(len(r))
    # 找出possible_road中的最短路径
    # 找出最短路径
    road_final=road_possible[road_length.index(min(road_length))]
    return road_final

for i in range(15):
    get_image()
map_origin=image_trans(image)
s=0

PBL=b'999'

while True:
    try:
        if PBL==0:
            time.sleep(0.1)
        if PBL==b'P21':
            # 对应题目的2.1问，首先要确立路径点，然后给串口屏发定位
            # time.sleep(1)
            while PBL!=b'000':
            #while True:
                try:
                    car_point,_=get_car_point()
                except:
                    pass
                if car_point!=last_car_point:
                    #send_order(car_point,ser_32)
                    #send_order(car_point,ser_32)
                    send_order(car_point,ser_32)
                    cv2.imwrite(f'{s}.jpg',image)
                    s=s+1
                    time.sleep(0.3)
                    last_car_point=car_point
        if PBL==b'P22':
            print('# 对应题目的2.2问，首先要确立路径点，然后给串口屏发定位')
            get_car_point()
            for i in range(10):
                _,image=capture.read()
                time.sleep(0.5)
            cv2.imwrite('P22.jpg', image)
            map_coverd=image_trans(image)
            grid_coverd_str, grid_coverd_int=get_grid_num(map_coverd)
            ##########这里可以有方案二########
            # 向stm32发送地图信息
            map22_now="M0123456789ABCDEF"
            print(grid_coverd_str, grid_coverd_int)
            for i in range(16):
                #print('正在获取图片信息'+str(i))
                map22_now='M'+map22_now[1:i+1]+grid_coverd_str[i]+map22_now[i+2:]
            pass
            # 开始路径规划
            while True:
                # 等待目标位置的发送
                if (aim_point22 in grid_coverd_int)&(aim_point22!=0):
                    time.sleep(1)
                    # 此时发送路径信息
                    send_order(map22_now,ser_32)
                    # 开始路径规划
                    road22_best=find_road(aim_point22, grid_coverd_int)
                    # 现在已经找到了最好的路径,发送给32
                    break
                time.sleep(0.1)
            # 发送路径信息
            time.sleep(1)
            path22='G'
            for i in road22_best:
                path22=path22+grid_str[i]
            send_order(path22+'Z',ser_32)
            # 开始小车运动，发送小车位置
            while PBL!=b'000':
                try:
                    car_point,_=get_car_point()
                except:
                    pass
                if car_point!=last_car_point:
                    send_order(car_point,ser_32)
                    cv2.imwrite(f'{s}.jpg',image)
                    s=s+1
                    time.sleep(0.3)
                    #send_order(car_point,ser_32)
                    #send_order(car_point,ser_32)
                    last_car_point=car_point
            pass
        if PBL==b'P23':
            # 首先我们需要识别新的地图数据
            get_car_point()
            for i in range(10):
                _,image=capture.read()
                time.sleep(0.5)
            cv2.imwrite('P23.jpg', image)
            map_change=image_trans(image)
            grid_change_str, grid_change_int=get_grid_num(map_change)
            # 向stm32发送地图信息
            print(grid_change_str)
            map23_now='M0123456789ABCDEF'
            for i in range(16):
                # print('正在获取图片信息'+str(i))
                map23_now='M'+map23_now[1:i+1]+grid_change_str[i]+map23_now[i+2:]
            # 开始路径规划
            # 首先还是转换为原先的地图
            grid_change_int_go=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
            for i in range(16):
                if grid_change_int[i]==16:
                    grid_change_int_go[i]=16
            while True:
                # 等待目标位置的发送
                if (aim_point23 in grid_change_int_go)&(aim_point23!=0):
                    # 找出aim_point23在grid_change_int中的位置
                    position = grid_change_int_go.index(aim_point23)
                    aim_point23=grid_change_int[position]
                    time.sleep(1)
                    send_order(map23_now,ser_32)
                    # 开始路径规划
                    road23_best=find_road(aim_point23, grid_change_int_go)
                    # 现在已经找到了最好的路径
                    break
                time.sleep(0.1)
            # 发送路径信息
            time.sleep(1)
            path23='G'
            for i in road23_best:
                path23=path23+grid_str[i]
            send_order(path23+'Z',ser_32)
            pass
            # 开始小车运动，发送小车位置
            while PBL!=b'000':
                try:
                    car_point,car_index=get_car_point()
                    car_point='C'+grid_int[grid_change_int[car_index]]
                except:
                    pass
                if car_point!=last_car_point:
                    send_order(car_point,ser_32)
                    cv2.imwrite(f'{s}.jpg',image)
                    s=s+1
                    time.sleep(0.3)
                    #send_order(car_point,ser_32)
                    #send_order(car_point,ser_32)
                    last_car_point=car_point
            pass
        else:
            time.sleep(0.1)
    except:
        print('Game Over')
        PBL=0
        last_car_point='C00'
        aim_point22=0
        aim_point23=0
        time.sleep(0.1)

'''
# 找出对应的黑色区域
black_cover=(map_origin-map_coverd)
# 建立两个集合用于存放地图信息
grid_coverd_str=[]
grid_coverd_int=[]
# 遍历每个区域，观察是否有遮挡
for i in range(16):
    x=int(i/4)*242
    y=int(i%4)*242
    b1=np.sum(black_cover[x:x+242,y:y+242]>100)
    # print(f'{i}:'+str(b1))
    if (b1>3000):
        grid_coverd_str.append('MBB')
    else:
        grid_coverd_str.append('M'+grid_int[i])
        grid_coverd_int.append(i)
'''