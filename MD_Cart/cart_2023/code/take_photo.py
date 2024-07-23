import cv2
capture=cv2.VideoCapture(0)
_,image=capture.read()
cv2.imwrite('14.jpg',image)
