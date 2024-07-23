'''try'''

# '''
import cv2

# 打开摄像头
cap = cv2.VideoCapture(0)  # 参数0表示使用默认摄像头，如果有多个摄像头，可以尝试更改参数

if not cap.isOpened():
  print("无法打开摄像头")
  exit()

while True:
  # 读取一帧
  ret, frame = cap.read()

  if not ret:
    print("无法获取帧")
    break

  # 获取帧率
  fps = cap.get(cv2.CAP_PROP_FPS)

  # 在帧上绘制帧率信息
  cv2.putText(frame, f'FPS: {int(fps)}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

  # 显示帧
  cv2.imshow("Frame", frame)

  # 按下 'q' 键退出循环
  if cv2.waitKey(1) & 0xFF == ord('q'):
    break

# 释放摄像头资源
cap.release()
cv2.destroyAllWindows()
#'''

'''
import cv2
import numpy as np

# 读取背景图片A和图形B
background_image = cv2.imread('back_gray.png')  # 请替换为实际的背景图片路径
template_image = cv2.imread('point.png')  # 请替换为实际的图形B路径

# 将图形B转换为灰度图像
template_gray = cv2.cvtColor(template_image, cv2.COLOR_BGR2GRAY)
back_gray = cv2.cvtColor(background_image, cv2.COLOR_BGR2GRAY)

# 使用模板匹配方法
result = cv2.matchTemplate(back_gray, template_gray, cv2.TM_CCOEFF_NORMED)

# 设定匹配的阈值
threshold = 0.8

# 获取匹配结果大于阈值的位置
locations = np.where(result >= threshold)
locations = list(zip(*locations[::-1]))  # 转换为(x, y)坐标

# 在背景图片上绘制矩形框标记匹配位置
for loc in locations:
  bottom_right = (loc[0] + template_image.shape[1], loc[1] + template_image.shape[0])
  cv2.rectangle(background_image, loc, bottom_right, (0, 255, 0), 2)

# 显示结果图像
cv2.imshow('Matching Result', background_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

def get_red_point(image,red_center):
    # _,image=capture.read()
    # 找到红色的点
    # image_red_true=image[:,:,1].astype(np.float32)
    # image_back=(image[:,:,2].astype(np.float32)+image[:,:,0].astype(np.float32))/2
    # image_back=image_back.astype(np.float32)
    # image_red=np.abs(image_red_true-image_back)
    # image_red=image[:,:,1]
    # image_red=image_red.astype(np.uint8)
    # red = np.array(np.argwhere(image_red>70))
    # red = np.array(np.argwhere(image_red>240))
    image1_red=image[:,:,2].astype(np.float32)
    image1_green=image[:,:,1].astype(np.float32)
    image1_blue=image[:,:,0].astype(np.float32)
    image2=image1_red-(image1_blue+image1_green)/2
    image2[image2<0]=0
    image2=image2*2
    red = np.array(np.argwhere(image2>70))
    # 红色中心点坐标
    if red.shape[0]>2:
        red_center=np.ones(2)
        red_center[0]=int(np.sum(red[:,0])/(np.size(red[:,0])+0.001))
        red_center[1]=int(np.sum(red[:,1])/(np.size(red[:,0])+0.001))
    else:
        pass
    return(red_center,image2.astype(np.uint8))
#'''


'''
import cv2
import time

# Number of frames to average over
avg_window = 10

# Open the default camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error opening camera")
    exit()

# Get the frame rate from the camera properties
fps = cap.get(cv2.CAP_PROP_FPS)

# Start a timer for calculating FPS
prev_time = time.time()
fps_list = []  # List to store FPS values for averaging

while True:
    # Capture a frame
    ret, frame = cap.read()

    if not ret:
        print("Can't receive frame (stream end?). Exiting...")
        break

    # Calculate FPS manually
    curr_time = time.time()
    fps_manual = 1 / (curr_time - prev_time)
    prev_time = curr_time

    # Calculate average FPS
    fps_list.append(fps_manual)
    if len(fps_list) > avg_window:
        fps_list.pop(0)
    avg_fps = sum(fps_list) / len(fps_list)

    # Draw average FPS on the frame
    cv2.putText(frame, f"FPS: {int(avg_fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow("Camera", frame)

    # Exit when 'q' is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# Release the video capture and close the window
cap.release()
cv2.destroyAllWindows()
#'''

'''
import cv2
import time

# Open the default camera (usually camera index 0)
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Get the frames per second (fps) of the camera
fps = cap.get(cv2.CAP_PROP_FPS)
print(f"Camera FPS: {fps}")

# Set font and text properties for displaying the frame rate
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 0.5
font_color = (255, 255, 255)
font_thickness = 1
text_position = (10, 30)

# Start capturing and processing frames
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Get the current time to calculate frame rate
    start_time = time.time()

    # Display the frame rate on the captured video stream
    cv2.putText(frame, f"FPS: {fps}", text_position, font, font_scale, font_color, font_thickness)

    # Show the frame
    cv2.imshow('Camera Feed', frame)

    # Check for the 'q' key to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Calculate the frame rate
    end_time = time.time()
    elapsed_time = end_time - start_time
    fps = 1 / elapsed_time

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
#'''

'''
import cv2
import time

# Function to calculate the average of a list
def calculate_average(lst):
    return sum(lst) / len(lst) if len(lst) > 0 else 0

# Open the default camera (usually camera index 0)
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Initialize variables for average frame rate calculation
fps_history = []
fps_average_window = 30  # Number of frames to consider for averaging

# Set font and text properties for displaying the frame rate
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 0.5
font_color = (255, 255, 255)
font_thickness = 1
text_position = (10, 30)

# Start capturing and processing frames
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Get the current time to calculate frame rate
    start_time = time.time()

    # Display the frame rate on the captured video stream
    cv2.putText(frame, f"FPS: {calculate_average(fps_history):.2f}", text_position, font, font_scale, font_color, font_thickness)

    # Show the frame
    cv2.imshow('Camera Feed', frame)

    # Check for the 'q' key to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    # Calculate the frame rate
    end_time = time.time()
    elapsed_time = end_time - start_time
    fps = 1 / elapsed_time

    # Append the current frame rate to the history
    fps_history.append(fps)

    # Keep only the last 'fps_average_window' frame rates in the history
    fps_history = fps_history[-fps_average_window:]

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
#'''

'''
import serial

# 打开串口
ser = serial.Serial('COM1', 9600)  # 请替换成你的串口和波特率

try:
  while True:
    # 读取串口数据
    data = ser.read(1)  # 读取一个字节的数据

    if data:
      # 将字节数据转换为16进制字符串并打印
      hex_data = data.hex().upper()
      print(f'Received: 0x{hex_data}')
except KeyboardInterrupt:
  # 在键盘输入Ctrl+C时关闭串口
  ser.close()
#'''
