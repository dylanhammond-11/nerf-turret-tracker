import cv2
import serial
import numpy as np
import struct

ser = serial.Serial('COM8', 115200)
myCamera = cv2.VideoCapture(0)

lowerRed1 = np.array([0, 100, 100])
upperRed1 = np.array([10,255,255])
lowerRed2 = np.array([160,100,100])
upperRed2 = np.array([179,255,255])

areaThreshold = 500 # Area needs to be this big to be an object

if not myCamera.isOpened():
    print("No camera found")
while True:
    ret,frame = myCamera.read()
    if not ret:
        print("unable to find frame")
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv,lowerRed1,upperRed1)
    mask2 = cv2.inRange(hsv,lowerRed2, upperRed2)
    mask = cv2.bitwise_or(mask1, mask2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    if contours:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        largest = contours[0]
        area = cv2.contourArea(largest)

        if area > areaThreshold:
            x, y, w, h = cv2.boundingRect(largest)
            cx = w//2 + x
            cy = h//2 + y
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            face_center = (x + w // 2, y + h // 2)
            coordinates = struct.pack('<hh', face_center[0], face_center[1])
            ser.write(coordinates)




    cv2.imshow("Frame", frame)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

myCamera.release()
cv2.destroyAllWindows()


