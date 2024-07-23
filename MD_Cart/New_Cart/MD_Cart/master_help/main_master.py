# 导入使用模块
import cv2
import numpy as np
import time
import serial
from Vision_Net import FastestDet

deep = FastestDet(drawOutput=False)
# 定义全局变量
# 定义小车的状态标志位
walking_flag = 0  # 停走标志位
has_goal = 0      # 是否设立了目标点的标志位
has_goal2 = 0
cross = 0         # 小车检测到路口，启动字符识别

# 以下 3 个状态需要 MCU 和 视觉进行交互通信进行判断，其他状态只需 视觉 自行分析切换
has_medicine = 1  # 小车是否装药的标志位,开始识别目标点
returning = 0     # 小车已经取药，开始返回，此时只进行巡线，不进行字符识别
iscrossing = 0    # 小车是否正在转弯的标志位
key_detector = 0  # 是否启动数字识别
set_order = 0     # 是否设已经设置了方向
time_count = 0    # 通过粗略估值判断小车是否停下
count_zero = 0
stopping = 0      # 小车是否停下的标志位
order = 9

goalNum = 1
go_cross = 0
catch_stop = True
back = 4

'''
针对每一帧图像，小车理论上只需要进行一种识别即可，即不需要同时巡线和字符识别
只有在检测到路口的情况下才需要进行字符识别，此时小车应该停止运动，只进行字符识别，不进行巡线
'''
goal = 0  # 定义目标房间位置
goal2 = 0
farlist = np.array(['0','0','0','0'])

image_width = 320
image_height = 240
line_upper = 40
line_lower = 110

order_list = np.array([])
return_order_list = np.array([])
# 红色区间1
red_upper_0 = np.array([10, 255, 255])
red_lower_0 = np.array([0, 43, 46])
# 红色区间2
red_upper_1 = np.array([180, 255, 255])
red_lower_1 = np.array([156, 43, 46])

def refreshFunc():
    global order_list,return_order_list 
    global has_goal,cross,set_order
    global walking_flag,time_count
    global has_medicine,count_zero
    global returning,stopping
    global iscrossing,order

    global go_cross
    global catch_stop
    global back
    global goal,goal2,farlist
    
    order_list = np.array([])
    return_order_list = np.array([])
    walking_flag = 0  
    has_goal = 0
    has_goal2 = 0
    cross = 0        
    has_medicine = 1  
    returning = 0    
    iscrossing = 0
    key_detector = 0 
    set_order = 0     
    count_zero = 0
    stopping = 0
    order = 9

    go_cross = 0
    catch_stop = True
    back = 4
    
    goal = 0
    # goal2 = 0
    # farlist = np.array(['0','0','0','0'])
# 循迹函数，计算车身到红线的位置差
# 传入函数的图像 img 大小为320 * 240，未经过任何图像预处理
# 对于一帧图像，小车应该区分出是直线、岔路还是停止区
# 返回值为 停走标志位(0/1) 位置环误差error 十字路口标志位
def tracking(img):
    global walking_flag,cross
    # 首先判断是否存在 红色 线路
    image_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask_0 = cv2.inRange(image_HSV, red_lower_1, red_upper_1)
    mask_1 = cv2.inRange(image_HSV, red_lower_0, red_upper_0)
    red_img = mask_1 + mask_0
    red_img = red_img[:120,:]
    k = np.ones((7, 7), np.uint8)
    red_img = cv2.morphologyEx(red_img, cv2.MORPH_OPEN, k)
    
    # cv2.imshow('red img',red_img)
    
    dect_area = red_img[line_upper:line_lower:4, :]
    # dect_area = cv2.morphologyEx(dect_area, cv2.MORPH_OPEN, k)  # 开运算
    
    # cv2.imshow('area',dect_area)
    
    array_ave_pos = np.array([])
    zero_num = 0
    for i in range(dect_area.shape[0]):
        test_line = dect_area[i]
        
        index = np.argwhere(test_line == 255)
        # print('index is:',index)
        # print('NO.',i,'index length:',len(index))
        if len(index) > 90:
            # 检测区域中出现丁字路口或者十字路口，需要停车进行方向的判断
            return 0,0,1
        elif len(index) <= 5:
            # 检测区域是正常的直行区域，计算位置环的误差
            zero_num += 1
            array_ave_pos = np.append(array_ave_pos, 0)
            if zero_num >= 14:
                # 检测到停止区，应该停车
                return 4, 0, 0
        else:
            index_sum = np.sum(index)
            ave_position = int(index_sum / len(index)) - 160
            array_ave_pos = np.append(array_ave_pos, ave_position)
    pos_error = np.sum(array_ave_pos) / len(array_ave_pos)
    return 1, int(pos_error),0

# 本函数是检测到十字路口时会调用的函数，如果没有检测到数据
# 则说明小车处于近端病房的路口
# 字符识别的函数，传入的图象是原始图像，在函数内部进行透视变换和畸变矫正
# 返回结果为 1(向左转) 2(继续直行) 3(向右转), 转角均为 90°
def detect_order(img):
    global farlist
    order_flag = 0
    out = deep.detect(img)
    num_count = len(out)
    if num_count:
        out.sort(key=lambda x:x[0][0])
        # print('*' * 30,'find the number','*' * 30)
        num_list = np.array([])
        for i in range(num_count):
            if out[i][1] == '1':
                num_list = np.append(num_list, '7')
            else:
                num_list = np.append(num_list, out[i][1])
        if len(num_list) == 4:
            farlist = num_list
        # print('*' * 30,'num list is:', num_list,'*' * 30)
        if str(goal) in num_list:
            mid = int(len(num_list) / 2)
            if str(goal) in num_list[0:mid]:
                order_flag = 1
            elif str(goal) in num_list[mid:]:
                order_flag = 3
        else:
            order_flag = 5
        # print('*' * 30,'order is:',order_flag,'*' * 30)
    else:
        # 路口没有检测到数字，说明在近端十字路口
        # print('*' * 30,'not find the number','*' * 30)
        # print('*' * 30,'goal is:',goal,'*' * 30)
        if goal == 1:
            order_flag = 1
        elif goal == 2:
            order_flag = 3
        else:
            order_flag = 5
        # print('*' * 30,'order is:',order_flag,'*' * 30)
    return order_flag


# 根据图像，识别图像中的数字作为目标点
# 返回值为目标点
def setGoal(img):
    # print('setting goal ing...')
    out = deep.detect(img)
    if len(out) == 1:  # 能从图片中提取到数字
        # cv2.imshow('goal test',img)
        # print('setting goal:',out)
        goal_point = out[0][2]  # 判断识别数字的置信度概率
        if goal_point > 0.85:
            return int(out[0][1])
        else:
            return 0
    else:
        has_goal = 0
        return 0


# 小车出于等待状态，此刻不需要进行任何操作
# mainly used for the situation when car has gotten the distination
# should clear all the flag that enabled by going() function
def waiting():
    # print('waiting for something...')
    set_order = 0
    iscrossing = 0
    stopping = 0
    return 0

# 小车已经送完药，准备返回
# 根据目标点，判断小车返回时需要转向的方向和转向次数
def backing():
    global iscrossing,set_order
    global return_order_list
    global order_list
    global has_medicine,returning,has_goal
    global back
    
    _, img = cap.read()  # 读取摄像头当前帧图像
    img = cv2.resize(img, (image_width, image_height))  # 将图像大小调整为合适尺寸
    
    flag,error_x,meet_cross = tracking(img)
    if iscrossing:
        return return_order_list[-1],error_x
    elif (not flag) and meet_cross:
        # cv2.imshow('back cross',img)
        # 小车检测到了十字路口
        if set_order:
            iscrossing = 1
            return return_order_list[-1],error_x
        else:
            if len(order_list) != 0:
                temp = order_list[-1]
            else:
                temp = 2
                
            if temp == 3:
                temp = 1
            elif temp == 1:
                temp = 3
            order_list = np.delete(order_list,-1)
            return_order_list = np.append(return_order_list,temp)
            # print('go:',order_list)
            # print('back:',return_order_list)
            set_order = 1
            iscrossing = 1
            return return_order_list[-1],error_x
    elif flag == 4:
        # print('get to the start area')
        # if back:
            # cv2.imshow(f'{back} start area',img)
            # back -= 1
        # meet the stop area
        refreshFunc()
        #print('Car has refreshed')
        return 6,0
    else:
        return 2,error_x
    
def going():
    global order_list,iscrossing,set_order,time_count,stopping
    global order,count_zero
    global go_cross,catch_stop
    
    _, img = cap.read()  # 读取摄像头当前帧图像
    img = cv2.resize(img, (image_width, image_height))  # 将图像大小调整为合适尺寸
    
    flag,error_x,meet_cross = tracking(img)
    
    if order == 0:
        if count_zero < 10:
            #if count_zero == 0:
                #print('start stopping')
            # print('trying to stop')
            count_zero += 1
            return 0,0
        #print('has stopped')
        stopping = 1
        count_zero = 0
        flag = 0
        meet_cross = 1
        
    if order == 4:
        if returning == 0:
            return 4,0
        
    # print('flag:',flag,'cross:',meet_cross)
    if iscrossing == 1:
        #print('AAAAA')
        return order_list[-1],error_x
    elif flag == 1 and (not meet_cross):
        #print('BBBBB')
        return 2,error_x
    elif (not flag) and meet_cross:
        if set_order:  # 已经设置了转弯方向
            #print('CCCCC')
            return order_list[-1],error_x
        else:          # 检测到路口，但是没有设置方向
            if stopping:
                # print('enter the detect function')
                direction = detect_order(img)
                # cv2.imshow(f'img to test {go_cross}',img)
                go_cross += 1
                if go_cross == 4:
                    go_cross = 0
                order_list = np.append(order_list,direction)  # 将方向保存到数组
                iscrossing = 1
                set_order = 1
                stopping = 0
                return order_list[-1],error_x
            else:
                set_order = 0
                if time_count >= 10:
                    # print('car begin stopping...')
                    time_count = 0
                    stopping = 1
                else:
                    # print('car not stop...')
                    time_count += 1
                    stopping = 0
                return 0,error_x
    elif (flag == 0 or flag == 4) and (not meet_cross):
        # print('meet the stop area')
        # meet the stop area
        if catch_stop:
            catch_stop = False
            # cv2.imshow('catch stop area',img)
        waiting()
        return 4,0
# 根据MCU发送过来的数据以及 tracking 函数的计算结果对小车运动模式进行判定
# 返回值为小车运动模式中的一种
# 小车总共模式有: 未放药、已放药、已取药
def decide_Car_Mode(img):
    global walking_flag,goal,goal2,has_goal,has_goal2
    # 如果没有装药，则什么也不需要做
    if (has_medicine == 0) and (returning == 0):
        # print('have nothing to do')
        waiting()
        return 0,0
    # 已经装药，但没有设置目标点，需要识别目标
    elif (not has_goal) or (not has_goal2):
        # print('setting goal ing')
        if goalNum == 1:
            has_goal2 = 1
        if not has_goal:
            # print('setting goal 1 ing')
            goal = setGoal(img)
            if goal != 0:
                has_goal = 1
        # print('goal1 is:', goal)
            
        if goalNum == 2 and goal != 0 and goal2 == 0:
            # print('setting goal 2 ing')
            goal2 = setGoal(img)
            if goal2 == 1:
                goal2 = 7
                
            if goal2 == goal:
                goal2 = 0
                has_goal2 = 0
                
            if goal2 != 0:
                has_goal2 = 1
            else:
                has_goal2 = 0
            # print('goal2 is:',goal2)
            # print(goalNum,goal,goal2,has_goal,has_goal2)
        return 9,0
    # 已经取药完成，开始返回
    elif returning:
        # print('now is backing')
        flag,error_x = backing()
        return flag,error_x
    # 已经装药，已经设置目标点，但没有取药，小车处于送药状态中
    else:
        # print('now is going')
        flag,error_x = going()
        return flag,error_x

# 已装药标志位 转弯中标志位 开始返回标志位  各位均是 1 表示有效 
def handleSerial():
    global has_medicine,iscrossing,returning,set_order
    global goalNum,goal2,farlist
    size = ser.inWaiting()  # 获得缓冲区字符
    # print('size = ',size)
    if size != 0:
        # print('*' * 30,'handle the serial','*' * 30)
        receive = ser.read(size).decode('utf-8')  # 读取内容并显示
        receive = receive[0:3]
        # print('infomation is:',receive)
        if receive == 'res':
            goalNum = 1
            refreshFunc()
            # print('refreshed')
        elif receive == 'pro':
            goalNum = 2
            goal2 = 0
            farlist = np.array(['0','0','0','0'])
            refreshFunc()
            # print('pro2 refreshed')
        if receive[0] == '1':
            has_medicine = 1
        elif receive[0] == '0':
            has_medicine = 0
            
        if receive[1] == '1':
            iscrossing = 0
            set_order = 0
            
        if receive[2] == '1':
            returning = 1
            set_order = 0
            iscrossing = 0
        ser.flushInput()  # 清空接收缓存区

# change int to str
def handleData(data):
    str_data = 0
    if data >= 0:
        if data > 999:
            data = 999
        str_data = '0' + '0' * (3 - len(str(data))) + str(data)
    else:
        if data < -999:
            data = -999
        str_data = '1' + '0' * (3 - len(str(-data))) + str(-data)
    
    return str_data

# 连接摄像头
ID = 0
cap = cv2.VideoCapture(ID) # ,cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))

connected = cap.isOpened()
ser = serial.Serial ("/dev/ttyAMA0", 115200)    #Open port with baud rate
# print('trying to connect...')
last_order = 9
while not connected:
    ID += 1
    cap = cv2.VideoCapture(ID)
    connected = cap.isOpened()

while connected:
    handleSerial()
    # ***** 图像预处理开始 *****
    _, frame = cap.read()  # 读取摄像头当前帧图像
    resized = cv2.resize(frame, (image_width, image_height))  # 将图像大小调整为合适尺寸
    order,error_pos = decide_Car_Mode(resized)
    str_error_pos = handleData(error_pos)
    # print('fps',fps)
    
    order = int(order)
    # if order != last_order:
    # print('order:',order)
    # last_order = order
    # if order != 2:
        # print('order is:',order)
    # print('iscrossing:',iscrossing)
    message = str(goal) + str(order) + str_error_pos
    str_far = '0'
    for i in range(4):
        str_far += farlist[i]
    str_far = str_far[1:5]
    ser.write((message + str(goal2) + str_far + 'a').encode('utf-8'))
    # print('*' * 50)
    # print('the message is:',message + str(goal2) + str_far)
    # if order != 0:
        # print('medicine:',has_medicine,'crossing:',iscrossing,'returning:',returning,'order:',order)
    # print('goal is:',goal,'order is:',order,'error_x is:',error_pos)
    # print('order list is:',order_list)
    # print('return order is:',return_order_list)
    # print('*' * 50,'\n')
    
    for i in range(line_upper,line_lower,4):
        cv2.line(resized, (0, i), (320,i), (0,0,255), 1, 4)
        
    # cv2.imshow('resized', resized)
    key = cv2.waitKey(1)
    if key == 27:
        break
    # print('finished')
    
cap.release()
cv2.destroyAllWindows()
