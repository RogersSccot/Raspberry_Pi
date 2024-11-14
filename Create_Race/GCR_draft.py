import numpy as np
import cv2
import math
import serial
import time
# from Vision_Net import FastestDet
from pyzbar import pyzbar

# 获取并处理图像
def get_image():
    # 开全局变量处理,分理处红绿蓝
    global image, image_red, image_green, image_blue
    # 从摄像头获取图像
    image=cv2.imread('C:\Parallel_World\BaiduSyncdisk\My_World\Software_World\Python\Raspberry_Pi\Create_Race\Hough_Circle_get_111.jpg')
    image_red=image[:,:,2].astype(np.float32)
    image_green=image[:,:,1].astype(np.float32)
    image_blue=image[:,:,0].astype(np.float32)
    #cv2.imshow('image',image)
    #cv2.waitKey(1)

# 处理命令的函数
def order_deal(order_temp):
    order_temp=min(max(-999,order_temp),999)
    if order_temp>=0:
        order_part='+'+str(int(int(order_temp)%1000/100))+str(int(int(order_temp)%100/10))+str(int(order_temp)%10)
    if order_temp<0:
        order_part='-'+str(int(int((-order_temp))%1000/100))+str(int(int((-order_temp))%100/10))+str(int((-order_temp))%10)
    return order_part

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
    print(aim_circle)
    return aim_circle[0:2]
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

# 在目标图像中寻找圆并找出圆心
def Hough_Circle_get():
    _img1 = image.copy()
    _img_gray1 = cv2.cvtColor(_img1, cv2.COLOR_BGR2GRAY)                        
    circle_center=np.zeros((3,2))
    # 进行中值滤波(这里我们略去)
    _dst_img1 = _img_gray1
    # 霍夫圆检测
    print("HoughCircles")
    circle = cv2.HoughCircles(_dst_img1, cv2.HOUGH_GRADIENT, 1, 150,param1=100, param2=70, minRadius=40, maxRadius=80)
    # 将检测结果绘制在图像上
    for i in circle[0, :]:  # 遍历矩阵的每一行的数据
        if (i[2] > 40)&(i[2] < 80):
            print(i[2])
            # 绘制圆形
            cv2.circle(_img1, (int(i[0]), int(i[1])), int(i[2]), (255, 0, 0), 10)
            # 绘制圆心
            cv2.circle(_img1, (int(i[0]), int(i[1])), 10, (255, 0, 0), -1)
    cv2.imwrite('HoughCircles.jpg',_img1)
    print(circle)
    return circle

print('Get_Image')
# 突出目标颜色
# 站在车的视角,从左到右依次为蓝,绿,红
# 进行霍夫圆检测
get_image()
_circle_now=Hough_Circle_get()
print("begin_judge")
# 判定检测是否合理，否则重新检测
while len(_circle_now[0, :])!=3:
    # 获取图像
    get_image()
    print('Get_Image_ing')
    _circle_now=Hough_Circle_get()
cv2.imwrite('Hough_Circle_get.jpg',image)
print('Get_Image3')
# 建立圆心集合
circle_center=np.zeros((3,2))
# 这里保存的点坐标分别是：红，绿，蓝
for i in range(3):
    print("i="+str(i))
    circle_center[i,0],circle_center[i,1]=get_aim_circle(_circle_now,str(i+1))
# 此时我们获取到了三个物料的位置,开始定位
print("Get_Image4")
# 开始计算相对误差
K_CJG,_=np.polyfit(circle_center[:,1], circle_center[:,0], 1)
# X_CJQ,Y_CJQ=circle_center[2,0]-320,circle_center[2,1]-150
X_CJQ,Y_CJQ=circle_center[1,0]-320,circle_center[1,1]-150
# 这里X表示左右，Y表示上下
dis_error=math.sqrt(X_CJQ**2+Y_CJQ**2)
print("X_CJQ"+str(X_CJQ))
print("Y_CJQ"+str(Y_CJQ))
print(dis_error)
time.sleep(1)
# 发送定位指令
print('K'+order_deal(np.arctan(K_CJG))+'X'+order_deal(-0.07*Y_CJQ)+'Y'+order_deal(-0.07*X_CJQ))
time.sleep(5)