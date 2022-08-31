import cv2

cap = cv2.VideoCapture("2022_06_10_01_00_-1.mp4")
length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
print( length )
