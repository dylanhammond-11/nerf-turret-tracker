import cv2
import serial
import time
import struct

ser = serial.Serial('COM8' , 115200)
time.sleep(2)
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades+ "haarcascade_frontalface_default.xml")

myCamera = cv2.VideoCapture(0)

if not myCamera.isOpened():
    print("Couldn't find your webcam")
    exit()


while True:
    ret, frame = myCamera.read()
    if not ret:
        print("Failed to make a frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)


    if len(faces) > 1:
        largest = sorted(faces, key=lambda b:b[2]*b[3], reverse=True)[0]
        x,y,w,h= largest
        cv2.rectangle(frame, (x,y), (x+w, y+h), (0,255,0),2)
        face_center = (x + w // 2, y + h // 2) # Finds coordinates of face center
        print(face_center)
        coordinates = struct.pack('<hh', face_center[0], face_center[1])
        ser.write(coordinates)
        #time.sleep(0.01)
        cv2.circle(frame, face_center, 5, (0, 0, 255), -1)  # red dot for face center

    cv2.imshow("Face Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


myCamera.release()
cv2.destroyAllWindows()

