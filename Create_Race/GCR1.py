import numpy as np
import cv2
import math
import serial
import time
from Vision_Net import FastestDet
from pyzbar import pyzbar

'''
本代码用于工创赛视觉部分,此为第一版本2024.9.12
核心任务及通信协议

STM32发给PI
STM32启动完毕:START
走到物料区:LWLQ(1,2)
走到加工区:LCJG(1,2)
走到暂存区:LZCQ(1,2)
1为红色,2为绿色,3为蓝色
识别二维码:SCANF

PI发给STM32:
识别结束后发送:QR+扫描结果
物料区开始抓取:CATCH(1,2,3)
粗加工开始抓取:CATL(1,2,3)
粗加工区开始放置(精度要求最高):
第1次层:PUTL(1,2,3)
第2次层:PUTH(1,2,3)
左移1格:MOVL(0,1,2)
右移1格:MOVR(0,1,2)
移动后自动开始定位
'''

#######################################################
# 外设初始化程序
# 打开摄像头,占用内存大,不轻易运行
# 摄像头0表示主摄像头,1表示侧摄像头
capture=cv2.VideoCapture(0)
capture_side=cv2.VideoCapture(1)
# 视觉神经网络先初始化,备用
loo_global=np.zeros((640,480,3),dtype=np.uint8)
deep = FastestDet(drawOutput=True)
# 打开串口
ser_32 = serial.Serial('/dev/ttyAMA0', 921600)
#######################################################
# 初始化全局变量
# 当前命令
PBL = 0
# 抓取循序(二维码读取结果)
QR_code=0
# 决定是否在放置时重复定位
put_locate=0
# 决定是否在抓取时重复定位
cat_locate=0

# 发送命令的指令
def send_order(order):
    print('order='+order)
    encoded_order = order.encode()
    ser_32.write(encoded_order)

# 处理命令的函数
def order_deal(order_temp):
    order_temp=min(max(-999,order_temp),999)
    if order_temp>=0:
        order_part='+'+str(int(int(order_temp)%1000/100))+str(int(int(order_temp)%100/10))+str(int(order_temp)%10)
    if order_temp<0:
        order_part='-'+str(int(int((-order_temp))%1000/100))+str(int(int((-order_temp))%100/10))+str(int((-order_temp))%10)
    return order_part

# 获取消息,通用函数
def get_mail(ser):
    encoded_mail = ser.read(1)
    print('mail='+encoded_mail)

# 获取并处理图像
def get_image():
    # 开全局变量处理,分理处红绿蓝
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
    image_red_only[image_red_only<0]=0
    return image_red_only

# 突出图像中的绿色目标
def get_green():
    global image, image_red, image_green, image_blue
    image_green_only=image_green*2-image_blue-image_red
    image_green_only[image_green_only<0]=0
    return image_green_only

# 突出图像中的蓝色目标
def get_blue():
    global image, image_red, image_green, image_blue
    image_blue_only=image_blue*2-image_blue-image_red
    image_blue_only[image_blue_only<0]=0
    return image_blue_only

# 突出目标颜色
def find_aim_color(aim_color):
    if aim_color == '1':
        return get_red()
    if aim_color == '2':
        return get_green()
    if aim_color == '3':
        return get_blue()
    else:
        return get_blue()

# 找到物料
def get_material():
    # 分别返回目标的行列
    material_XY=0,0
    print('get_material函数未完善,暂时不需要')
    return material_XY

# 找到位置
def get_position():
    # 分别返回目标的行列
    position_XY=0,0
    print('get_position函数未完善')
    return position_XY

# 定位物料位置的函数
def locate_aim_material(aim_image,image):
    # 这里K表示车的倾斜度,X表示横向误差,Y表示纵向误差
    dis_error, order=100,'K+000X+000Y+000'
    print('locate_aim_material函数未完善,暂时不需要')
    pass

# 定位目标位置的函数
def locate_aim_position(aim_image,image):
    # 这里K表示车的倾斜度,X表示横向误差,Y表示纵向误差
    dis_error, order=100,'K+000X+000Y+000'
    print('locate_aim_position函数未完善,暂时不需要')
    pass

# 识别二维码,对二维码进行编译,返回值
def decode_qr_code(QR_img):
    return pyzbar.decode(QR_img, symbols=[pyzbar.ZBarSymbol.QRCODE])

# 判断在物料区当前视角中是否有是目标物料
def Judge_WLQ_material(aim_image):
    _right_color=np.sum(aim_image[240:400,240:400])
    return _right_color>900000

def Move_Color(dis1):
    if dis1>=0:
        send_order('MOVR'+str(abs(dis1)))
    else:
        send_order('MOVL'+str(abs(dis1)))
    pass

# 在目标图像中寻找圆并找出圆心
def Hough_Circle_get():
    _img1 = image.copy()
    _img_gray1 = cv2.cvtColor(_img1, cv2.COLOR_BGR2GRAY)                        
    circle_center=np.zeros((3,2))
    # 进行中值滤波(这里我们略去)
    _dst_img1 = _img_gray1
    # 霍夫圆检测
    circle = cv2.HoughCircles(_dst_img1, cv2.HOUGH_GRADIENT, 1, 150,param1=100, param2=70, minRadius=0, maxRadius=10000)
    print(PBL+':'+circle)
    return circle

# 此时获得的结果分别为圆心的x,y坐标和半径
def get_aim_circle(circles,aim_color):
    circle_possible=np.zeros((3))
    # 遍历所有圆心，找到对应颜色的圆心
    for j in range(len(circles[0, :])):
        i=circles[0, j]
        if (i[2] > 40)&(i[2] < 80):
            # 在此处绘制正方形的长宽
            R=70
            top_left_corner=(int(i[0])-R, int(i[1])-R)
            bottom_right_corner=(int(i[0])+R, int(i[1])+R)
            # 提取目标颜色
            _color_only_temp=find_aim_color(aim_color)
            circle_possible[j]=np.sum(_color_only_temp[top_left_corner[1]:bottom_right_corner[1], top_left_corner[0]:bottom_right_corner[0]])
    # 找到目标颜色的圆心
    aim_circle=circles[0, np.argmax(circle_possible)]  
    return aim_circle

#######################################################
# 主控程序
#######################################################

# 外界大循环保证程序报错时依旧可以继续运行
while True:
    try:
        while True:
            PBL = ser_32.read(5)
            PBL=PBL.decode('utf-8')
            # 等待STM32发送控制指令给我,执行具体的任务,这里并不需要双线程,也不需要记录上位机
            # 收到启动信号时,点灯
            if PBL=='START':
                send_order('OKOK')
            
            # 扫描二维码,并生成抓取顺序
            if PBL == 'SCANF':
                _,QR_img=capture_side.read()
                # 获取二维码结果
                QR_results = decode_qr_code(QR_img)
                print("正在解码")
                if len(QR_results):
                    print("解码结果:")
                    QR_results=QR_results[0].data.decode("utf-8")
                    print(QR_results)
                else:
                    print("无法识别:")
                    QR_results='213+231'
                # 发送二维码结果给STM32
                send_order('QR'+QR_results)
                # 此时我们已获得二维码结果,分析抓取顺序
                QR1,QR2=QR_code.split('+')
                send_order('OKOK')
            
            # 此时是定位指令,必须
            if PBL[0]=='L':
                # 此时是定位物料区,开始判断物料颜色
                if PBL[0:4]=='LWLQ':
                    goods_num=0
                    while goods_num<3:
                        # 刷新图像
                        get_image()
                        # 获取物料颜色
                        if PBL[0:5]=='LWLQ1':
                            aim_color=QR1[goods_num]
                        else:
                            aim_color=QR2[goods_num]
                        # 判断目标位置中有无物料
                        if Judge_WLQ_material(find_aim_color(aim_color)):
                            # 抓取物料
                            send_order('CATCH'+aim_color)
                            goods_num+=1
                        else:
                            time.sleep(0.1)
                    # 此时已抓完物料
                    send_order('OKOK')
                
                # 此时是定位加工区,这里放的时候不需要顺序,但是拿的时候需要顺序
                if PBL[0:4]=='LCJG':
                    # 首先进行校准
                    dis_error = 100
                    while dis_error>10:
                        # 获取图像
                        get_image()
                        # 突出目标颜色
                        # 站在车的视角,从左到右依次为蓝,绿,红
                        # 进行霍夫圆检测
                        _circle_now=Hough_Circle_get()
                        # 判定检测是否合理，否则重新检测
                        while len(_circle_now[0, :])!=3:
                            _circle_now=Hough_Circle_get()
                        # 建立圆心集合
                        circle_center=np.zeros((3,2))
                        for i in range(3):
                            circle_center[i,0],circle_center[i,1]=get_aim_circle(_circle_now,str(i+1))
                        # 此时我们获取到了三个物料的位置,开始定位
                        K_CJG,_=np.polyfit(circle_center[:,1], circle_center[:,0], 1)
                        X_CJQ,Y_CJQ=circle_center[1,1]-320,circle_center[1,0]-240
                        dis_error=math.sqrt(X_CJQ**2+Y_CJQ**2)
                        # 发送定位指令
                        send_order('K'+order_deal(K_CJG)+'X'+order_deal(X_CJQ)+'Y'+order_deal(Y_CJQ))
                    # 此时已经定位完毕,开始放置物料,首先需要将车移动到正确的位置
                    # 记录下当前的位置
                    Location_Now=2
                    for goods_num in range(3):
                        if PBL[0:5]=='LCJG1':
                            aim_color=QR1[goods_num]
                            put_order_LCJG='PUTL'+aim_color
                        else:
                            aim_color=QR2[goods_num]
                            put_order_LCJG='PUTL'+aim_color
                        Move_Dis=int(aim_color)-Location_Now
                        Move_Color(Move_Dis)
                        Location_Now=int(aim_color)
                        '''
                        这里是否需要定位暂时待定
                        '''
                        if put_locate==1:
                            dis_error = 100
                            while dis_error>10:
                                # 获取图像
                                get_image()
                                # 突出目标颜色
                                # 站在车的视角,从左到右依次为蓝,绿,红
                                # 进行霍夫圆检测
                                _circle_now=Hough_Circle_get()
                                # 获取目标圆心
                                _circle_center=get_aim_circle(_circle_now,aim_color)
                                # 此时我们获取到了三个物料的位置,开始定位
                                K_CJG=0
                                X_CJQ,Y_CJQ=_circle_center[0]-320,_circle_center[1]-240
                                dis_error=math.sqrt(X_CJQ**2+Y_CJQ**2)
                                # 发送定位指令
                                send_order('K'+order_deal(K_CJG)+'X'+order_deal(X_CJQ)+'Y'+order_deal(Y_CJQ))
                            pass
                        # 放置物料
                        send_order(put_order_LCJG)
                        time.sleep(3)
                    # 此时已放置完物料,接下来我们需要取走物料
                    for goods_num in range(3):
                        if PBL[0:5]=='LCJG1':
                            aim_color=QR1[goods_num]
                            catch_order_LCJG='CATL'+aim_color
                        else:
                            aim_color=QR2[goods_num]
                            catch_order_LCJG='CATL'+aim_color
                        Move_Dis=int(aim_color)-Location_Now
                        Move_Color(Move_Dis)
                        Location_Now=int(aim_color)
                        # 取走物料
                        send_order(catch_order_LCJG)
                        '''
                        这里是否需要定位暂时待定
                        '''
                        if cat_locate==1:
                            dis_error = 100
                            while dis_error>10:
                                # 获取图像
                                get_image()
                                # 突出目标颜色
                                # 站在车的视角,从左到右依次为蓝,绿,红
                                # 进行霍夫圆检测
                                _circle_now=Hough_Circle_get()
                                # 获取目标圆心
                                _circle_center=get_aim_circle(_circle_now,aim_color)
                                # 此时我们获取到了三个物料的位置,开始定位
                                K_CJG=0
                                X_CJQ,Y_CJQ=_circle_center[0]-320,_circle_center[1]-240
                                dis_error=math.sqrt(X_CJQ**2+Y_CJQ**2)
                                # 发送定位指令
                                send_order('K'+order_deal(K_CJG)+'X'+order_deal(X_CJQ)+'Y'+order_deal(Y_CJQ))
                            pass
                        time.sleep(3)

                # 此时是定位暂存区
                if PBL[0:4]=='LZCQ':

                    # 首先进行校准
                    dis_error = 100
                    while dis_error>10:
                        # 获取图像
                        get_image()
                        # 突出目标颜色
                        # 站在车的视角,从左到右依次为蓝,绿,红
                        # 进行霍夫圆检测
                        _circle_now=Hough_Circle_get()
                        # 判定检测是否合理，否则重新检测
                        while len(_circle_now[0, :])!=3:
                            _circle_now=Hough_Circle_get()
                        # 建立圆心集合
                        circle_center=np.zeros((3,2))
                        for i in range(3):
                            circle_center[i,0],circle_center[i,1]=get_aim_circle(_circle_now,str(i+1))
                        # 此时我们获取到了三个物料的位置,开始定位
                        K_CJG,_=np.polyfit(circle_center[:,1], circle_center[:,0], 1)
                        X_CJQ,Y_CJQ=circle_center[1,1]-320,circle_center[1,0]-240
                        dis_error=math.sqrt(X_CJQ**2+Y_CJQ**2)
                        # 发送定位指令 
                        send_order('K'+order_deal(K_CJG)+'X'+order_deal(X_CJQ)+'Y'+order_deal(Y_CJQ))
                    # 此时已经定位完毕,开始放置物料,首先需要将车移动到正确的位置
                    # 记录下当前的位置
                    Location_Now=2
                    for goods_num in range(3):
                        if PBL[0:5]=='LZCQ1':
                            aim_color=QR1[goods_num]
                            put_order_LZCQ='PUTL'+aim_color
                        else:
                            aim_color=QR2[goods_num]
                            put_order_LZCQ='PUTH'+aim_color
                        Move_Dis=int(aim_color)-Location_Now
                        Move_Color(Move_Dis)
                        Location_Now=int(aim_color)
                        # 放置物料
                        send_order(put_order_LZCQ)
                        '''
                        这里是否需要定位暂时待定
                        '''
                        if put_locate==1:
                            dis_error = 100
                            while dis_error>10:
                                # 获取图像
                                get_image()
                                # 突出目标颜色
                                # 站在车的视角,从左到右依次为蓝,绿,红
                                # 进行霍夫圆检测
                                _circle_now=Hough_Circle_get()
                                # 获取目标圆心
                                _circle_center=get_aim_circle(_circle_now,aim_color)
                                # 此时我们获取到了三个物料的位置,开始定位
                                K_CJG=0
                                X_CJQ,Y_CJQ=_circle_center[0]-320,_circle_center[1]-240
                                dis_error=math.sqrt(X_CJQ**2+Y_CJQ**2)
                                # 发送定位指令
                                send_order('K'+order_deal(K_CJG)+'X'+order_deal(X_CJQ)+'Y'+order_deal(Y_CJQ))
                            pass
                        time.sleep(3)

            # 更新STM32指令
            PBL=0
            pass
    except:
        PBL=0
        pass



