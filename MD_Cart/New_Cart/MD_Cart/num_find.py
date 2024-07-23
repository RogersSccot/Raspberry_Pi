import cv2
import numpy as np
import time
from Vision_Net import FastestDet
import threading

# cap = cv2.VideoCapture(1)
loo_global=np.zeros((640,480,3),dtype=np.uint8)

# def updatephoto():
#     global loo_global
#     while True:
#         map,loo_global=cap.read()
# t=threading.Thread(target=updatephoto)
# t.start()
# lock=threading.Lock()

# def photo_processing():
#     #读取图像
#     lock.acquire()
#     frame=loo_global
#     lock.release()
#     #roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]   #对原始图像进行裁切
#     return frame
# photo = photo_processing()

deep = FastestDet(drawOutput=True)
# 连接摄像头

#cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
# connected = cap.isOpened()
print('trying to connect...')
start_time = 0

# while connected:
# _, frame = cap.read()  # 读取摄像头当前帧图像
frame=cv2.imread('C:\Parallel_World\BaiduSyncdisk\My_World\Software_World\Python\Computer_Vision\Med_Del_Cart\image.png')
# cv2.imshow('origin',frame)
out = deep.detect(frame)
if out:
    print(out)
else:
    print('no proper image')
end_time = time.time()
spend_time = (end_time - start_time) * 1000
start_time = end_time
print('time is:',spend_time,' ms')
cv2.imwrite('C:\Parallel_World\BaiduSyncdisk\My_World\Software_World\Python\Computer_Vision\Med_Del_Cart\Result1.png',frame)

# cap.release()
# cv2.destroyAllWindows()
# # import numpy as np

# # [((259.5, 330.5), '6', 0.9501334592976022),
# #  ((238.0, 119.5), '3', 0.7888560051136942),
# #  ((188.5, 197.5), '3', 0.6053135649194342)]

# # lis = [((281.5, 304.0), '3', 0.6421383037056354)]
# # print('length is: ',len(lis))
# # print('list 0 is: ',lis[0])
# # print('the second num of list[0] is: ',lis[0][1])
