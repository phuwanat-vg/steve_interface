import cv2 

cap = cv2.VideoCapture(0)

while True:
    ret,frame = cap.read()

    cv2.imshow("Original", frame)
    cv2.waitKey(1)
