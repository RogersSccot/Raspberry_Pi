import numpy as np
import serial
import cv2
import numpy as np
import time

# 打开摄像机和端口，不轻易运行
ser_32 = serial.Serial('/dev/ttyAMA0', 115200)
ser_BT = serial.Serial('/dev/ttyAMA3', 9600)

def send_order(order,ser):
    print('order:'+order)
    encoded_data = order.encode()
    ser.write(encoded_data)

# 获取蓝牙信号
def get_BT():
    hex_data = ser_BT.read(1)
    return hex_data

def get_32():
    hex_data = ser_32.read(1)
    return hex_data

def get_feed(ser):
    hex_data = ser.read(1)
    return hex_data

# send_order('901S', ser_32)

# time.sleep(5)

# send_order('L+00', ser_32)

#send_order('901S', ser_32)
#time.sleep(8)
#send_order('L+00', ser_32)
# time.sleep(1)

#send_order('901S', ser_32)
#time.sleep(5)
send_order('L+90', ser_32)
# time.sleep(5)
# send_order('V+00', ser_32)
