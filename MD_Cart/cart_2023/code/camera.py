import cv2
import numpy as np
import matplotlib.pyplot as plt
capture = cv2.VideoCapture(0)
while True:
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
    print('sides'+str(sides_250))
    print('above'+str(above_250))
    cv2.imshow('image', road)
    cv2.waitKey(1)