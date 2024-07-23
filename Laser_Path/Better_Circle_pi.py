# prepare the library
import cv2
import numpy as np
import math
import matplotlib.pyplot as plt
import serial
from PIL import Image
import pylab as pl
from scipy import ndimage
import scipy.ndimage
import time
from matplotlib.patches import Polygon

# open the camera and serial port
capture = cv2.VideoCapture(1)
_, image = capture.read()
gray = np.zeros((480, 640))
# ser.close()
ser = serial.Serial('COM10', 115200)
# ser = serial.Serial('/dev/ttyAMA0', 115200)
# 这是用来发送给电机数据的
# ser_out=serial.Serial('COM10', 115200)

# define function
# 定义字符串的转换方式
# 先定义最大速度
max_speed = 10
def convert_to_two_digits(number):
    if 10 > number >= 0:
        return '+0' + str(number)
    if 100 > number >= 10:
        return '+' + str(min(number, max_speed))
    if 0 > number > -10:
        return '-0' + str(number)[1]
    if -10 >= number > -100:
        return '-' + str(min(abs(number), max_speed))

# 在整个图片中获取红色点
# 在整个image所获取的图片中[:,:,1]是绿色图片,[:,:,2]是红色图片
template_image = cv2.imread('point.png')
template_gray = cv2.cvtColor(template_image, cv2.COLOR_BGR2GRAY)
threshold = 0.8
def get_red_point(capture, red_center):
    _, image = capture.read()
    image1_red = image[:, :, 2].astype(np.float32)
    image1_green = image[:, :, 1].astype(np.float32)
    image1_blue = image[:, :, 0].astype(np.float32)
    image2 = image1_red - (image1_blue + image1_green) / 2
    image2[image2 < 0] = 0
    image2 = image2 * 2
    back_gray = image2.astype(np.uint8)
    try:
        result = cv2.matchTemplate(back_gray, template_gray, cv2.TM_CCOEFF_NORMED)
        locations = np.where(result >= threshold)
        locations = list(zip(*locations[::-1]))  # 转换为(x, y)坐标
        red_center = np.array([int(locations[0][1] + template_image.shape[0] / 2), int(locations[0][0] + template_image.shape[1] / 2)])
    except cv2.error:
        red_center = red_center
    return (red_center)

def get_green_point(capture, green_center):
    _,image=capture.read()
    image1_red=image[:,:,2].astype(np.float32)
    image1_green=image[:,:,1].astype(np.float32)
    image1_blue=image[:,:,0].astype(np.float32)
    image2=image1_green-(image1_blue+image1_red)/2
    image2[image2<0]=0
    image2=image2*2
    back_gray=image2.astype(np.uint8)
    try:
        result = cv2.matchTemplate(back_gray, template_gray_1, cv2.TM_CCOEFF_NORMED)
        locations = np.where(result >= threshold)
        locations = list(zip(*locations[::-1]))  # 转换为(x, y)坐标
        green_center=np.array([int(locations[0][1] + template_image_1.shape[0] / 2),int(locations[0][0] + template_image_1.shape[1] / 2)])
    except cv2.error:
        green_center=green_center
    return(green_center)

def move_to_the_point(dis_aim, speed, aim_center, sleep_time, red_center):
    dis_center = 20
    speed = speed / 10
    # while True:
    while dis_center > dis_aim:
        red_center = get_red_point(capture, red_center)
        dis_center = np.linalg.norm(aim_center - red_center)
        rotate = (aim_center - red_center) * speed
        rotate = rotate * np.array([1, -1])
        # 第1个数正上负下  第2个数，正左负右
        data = convert_to_two_digits(int(rotate[0])) + convert_to_two_digits(int(rotate[1]))
        encoded_data = data.encode()
        ser_out.write(encoded_data)
        time.sleep(sleep_time)

# 蜂鸣器响3秒
def buzzer_open():
    data = "ffffff"
    encoded_data = data.encode()
    ser_out.write(encoded_data)

# 定义卷积核
kernel = np.array([
[0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0],
[0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0],
[0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0],
[0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0],
[0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0],
[0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0],
[0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0],
[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
[1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1],
[0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0],
[0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0],
[0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0],
[0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0],
[0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0],
[0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0],
[0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0],
])*0.01
a=11
b=11
border_point=np.ones((4,2))
red_center=np.ones(2)
red_center[0]=240
red_center[1]=320
green_center=np.ones(2)
green_center[0]=240
green_center[1]=320

# we continue receiving the signal from the serial
while True:
    data = ser.read(1).hex().upper()
    if data==1:
        print('break all')
        break
    if data==2:
        while True:
            change = int(ser.read(1).hex().upper())
            if change == 0:
                print('break point')
                break
            if change == 1:
                i = int(ser.read(1).hex().upper()) - 1
                while True:
                    OK = 0
                    # 分别读取4个边界点的坐标
                    while OK == 0:
                        _, image = capture.read()
                        # 对于所获得的图像，首先计算他的几何中心
                        # 先转为灰度图
                        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                        # edges_show=cv2.Canny(gray, 240, 200)
                        edges_show = gray
                        # 获取激光点
                        red_center = get_red_point(capture, red_center)
                        if (int(red_center[0] - a) > 0) & (int(red_center[0] + b) < 480) & (
                                int(red_center[1] - a) > 0) & (int(red_center[1] + b) < 640):
                            edges_show[int(red_center[0] - a):int(red_center[0] + b),
                            int(red_center[1] - a):int(red_center[1] + b)] = kernel * 25400
                        else:
                            edges_show[240 - a:240 + b, 320 - a:320 + b] = kernel * 25400
                        cv2.imshow('edges_show', edges_show)
                        cv2.waitKey(1)
                        OK = int(int(ser.read(1).hex().upper()))
                        if OK == 1:
                            border_point[i, :] = red_center
                    break
        # 首先计算整个图形的中心点
        circle_center = np.ones(2)
        circle_center[0] = int(np.sum(border_point[:, 0]) / (np.size(border_point[:, 0]) + 0.001))
        circle_center[1] = int(np.sum(border_point[:, 1]) / (np.size(border_point[:, 1]) + 0.001))
        ad = cv2.imread('ad.png')
        square_origin = cv2.imread('square_origin.png')
        circle_origin = cv2.imread('circle_origin.png')
        # 原先的点
        pts = np.float32([[0, 0], [0, square_origin.shape[1]], [square_origin.shape[0], square_origin.shape[1]],
                          [square_origin.shape[0], 0]])
        # # 后面的点
        # 由于变换的特殊性，这里后面的点是列行，反着来的
        border_point_1 = np.zeros(np.shape(border_point))
        border_point_1[:, 0] = border_point[:, 1]
        border_point_1[:, 1] = border_point[:, 0]
        pts1 = np.float32(border_point_1)
        # 生成最为关键的变换矩阵
        M = cv2.getPerspectiveTransform(pts, pts1)
        # 变换正方形
        square_trans = cv2.warpPerspective(square_origin, M, (ad.shape[1], ad.shape[0]))
        square_trans = cv2.cvtColor(square_trans, cv2.COLOR_RGB2GRAY)
        _, square_trans = cv2.threshold(square_trans, 127, 255, cv2.THRESH_BINARY)
        # 变换圆
        circle_trans = cv2.warpPerspective(circle_origin, M, (ad.shape[1], ad.shape[0]))
        circle_trans = cv2.cvtColor(circle_trans, cv2.COLOR_RGB2GRAY)
        _, circle_trans = cv2.threshold(circle_trans, 127, 255, cv2.THRESH_BINARY)
        # ----------------------------------------------------------------------------------

        # 然后获取每条直线的斜率（方向）,范围是0到2*pi
        k_angle = np.array(range(0, int(200 * (2 * math.pi)), 5)) / 200 - math.pi
        # 获取边框上的点，用于之后遍历路径
        stripe = np.array(np.where(square_trans))
        stripe_center = circle_center
        # 获取整个图面的中心点
        # 所有的点都是以行列的形式标注的，先取行，再取列
        square_trans[int(stripe_center[0]), int(stripe_center[1])] = 255
        # 接下来找出每个有值的点所具有对应三角值
        # 找出相对位置
        k_re = stripe[0, :] - stripe_center[0], stripe[1, :] - stripe_center[1]
        k_re = np.array(k_re)
        k_stripe_pi = np.arctan2(k_re[0, :], k_re[1, :])
        # 找出所有的斜率对应的最接近的点
        # 将找出的边界点存入对应的road_point矩阵中,最后保存为square_point
        # road用于显示最后的路径图像
        # road_point收集有顺序的点坐标
        road = np.zeros(np.shape(gray))
        road_point = np.ones((np.size(k_angle), 2))
        for k in range(0, np.size(k_angle)):
            k_rela = np.abs(k_stripe_pi - k_angle[k])
            min_index = np.argmin(k_rela)
            # road[int(stripe[0, min_index]), int(stripe[1, min_index])] = 255
            road_point[k, :] = [int(stripe[0, min_index]), int(stripe[1, min_index])]
        square_point_num = np.shape(road_point)[0]
        # 调整正方形的顺序
        square_point = np.zeros(np.shape(road_point))
        square_point[0:int(square_point_num / 8 * 7), :] = road_point[(int(square_point_num) - int(
            square_point_num / 8 * 7)) - 1:square_point_num - 1, :]
        square_point[int(square_point_num / 8 * 7) - 1:int(square_point_num) - 1, :] = road_point[0:int(
            square_point_num - int(square_point_num / 8 * 7)), :]

        # -------------------------------------------------------------------

        # 获取边框上的点，用于之后遍历路径
        stripe = np.array(np.where(circle_trans))
        # 获取整个图面的中心点
        # 所有的点都是以行列的形式标注的，先取行，再取列
        circle_trans[int(stripe_center[0]), int(stripe_center[1])] = 255
        # 接下来找出每个有值的点所具有对应三角值
        # 找出相对位置
        k_re = stripe[0, :] - stripe_center[0], stripe[1, :] - stripe_center[1]
        k_re = np.array(k_re)
        k_stripe_pi = np.arctan2(k_re[0, :], k_re[1, :])
        # 找出所有的斜率对应的最接近的点
        # 将找出的边界点存入对应的road_point矩阵中,最后保存为square_point
        # road用于显示最后的路径图像
        # road_point收集有顺序的点坐标
        # road=np.zeros(np.shape(gray))
        road_point = np.ones((np.size(k_angle), 2))
        for k in range(0, np.size(k_angle)):
            k_rela = np.abs(k_stripe_pi - k_angle[k])
            min_index = np.argmin(k_rela)
            # road[int(stripe[0, min_index]), int(stripe[1, min_index])] = 255
            road_point[k, :] = [int(stripe[0, min_index]), int(stripe[1, min_index])]
        circle_point_num = np.shape(road_point)[0]
        # 调整圆的顺序
        circle_point = np.zeros(np.shape(road_point))
        circle_point[0:int(circle_point_num / 8 * 7), :] = road_point[(int(circle_point_num) - int(
            circle_point_num / 8 * 7)) - 1:circle_point_num - 1, :]
        circle_point[int(circle_point_num / 8 * 7) - 1:int(circle_point_num) - 1, :] = road_point[0:int(
            circle_point_num - int(circle_point_num / 8 * 7)), :]

        # -------------------------------------------------------------------
    if data==3:
        # move_to_the_point(dis_aim(要求的最终精度),speed(1~10),sleep_time(暂停的时间),aim_center(最终的目标点))
        # 现将点移动至中心点
        move_to_the_point(5, 3, circle_center, 0.1, red_center)
        buzzer_open()
    if data==4:
        # 遍历所有的提取出来的点,这里是画正方形
        # move_to_the_point(dis_aim(要求的最终精度),speed(1~10),aim_center(最终的目标点),sleep_time(暂停的时间))
        for point in range(0, np.size(k_angle) - 1):
            point_center = square_point[point, :]
            move_to_the_point(10, 3, point_center, 0.1, red_center)
        buzzer_open()
    if data == 5:
        # 遍历所有的提取出来的点
        # 这里是画圆
        # move_to_the_point(dis_aim(要求的最终精度),speed(1~10),aim_center(最终的目标点),sleep_time(暂停的时间))
        for point in range(0, np.size(k_angle)):
            point_center = circle_point[point, :]
            move_to_the_point(12, 3, point_center, 0.1, red_center)
        buzzer_open()
    if data == 5:
        # 绿激光
        green_center = get_green_point(capture, green_center)
        red_center = get_green_point(capture, red_center)
        dis_center = np.linalg.norm(green_center - red_center)
        while dis_center > 3:
            green_center = get_green_point(capture, green_center)
            red_center = get_green_point(capture, red_center)
            move_to_the_point(12, 3, green_center, 0.1, red_center)


