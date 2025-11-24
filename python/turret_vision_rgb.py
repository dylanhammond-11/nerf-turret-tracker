# Python vision code for implementation with 3d printed nerf turret see https://dylanhammond-11.github.io/projects/auto-nerf-turret/index/
# Use alongside turret_color.ino for full turret implementation
# Code connects to external camera and detects multiple rgb colored objects. Center coordinates of objects are
# sent via serial to microcontroller code to process and track
# NOTE: This code is currently still in development alongside turret_color.ino and isn't fully functional
# Code by Dylan Hammond

import cv2
import serial
import numpy as np
import time
import struct
import math
ser = serial.Serial('COM7', 115200)

# HSV color ranges (red wraps around range)
hsvColors = {
    "Red": [np.array([0,100,100]), np.array([10,255,255]),
            np.array([160,100,100]), np.array([179,255,255])],
    "Green":[np.array([40,70,70]), np.array([80,255,255])],
    "Blue": [np.array([100,150,0]), np.array([140,255,255])]
}

areaThreshold = 500 # Area needs to be this big to be an object
# RGB Color ranges for making a rectangle
rgbColors = {
    'Red': (0,0,255),
    'Blue': (0,255,0),
    'Green': (255,0,0)
}

myCamera = cv2.VideoCapture(0)



targets = [] # List of tuples for "color", x, y of balloons
currentTargetIndex = 0

def findBalloons(frame):
    detections = { "red": [], "green": [],"blue":[]} # Dictionary with colors and there cx cy coordinates
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    for hsvColor, hsvRange in hsvColors.items():
        if hsvColor == "Red":
            mask1 = cv2.inRange(hsv, hsvRange[0], hsvRange[1])
            mask2 = cv2.inRange(hsv, hsvRange[2], hsvRange[3])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, hsvRange[0], hsvRange[1])

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > areaThreshold:
                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w // 2
                cy = y + h // 2
                detections[hsvColor].append((cx,cy))
                cv2.rectangle(frame, (x, y), (x + w, y + h), rgbColors[hsvColor])



    return detections
def distance(p1,p2):
    targetDistance = sqrt


if not myCamera.isOpened():
    print("Couldn't find a camera")
    exit()

while True:
    ret, frame = myCamera.read()
    if not ret:
        print("failed to find a frame")
        break
    # clear coordinates list, ready to store
    redCoordinates = []
    greenCoordinates = []
    blueCoordinates = []




    cv2.imshow("Color Detection", frame)

    def convertCoordinates(coords):
        return ','.join([f"{x},{y}" for x,y in coords])

    coordString = f"R:{convertCoordinates(redCoordinates)};G:{convertCoordinates(greenCoordinates)};B:{convertCoordinates(blueCoordinates)}\n"
    ser.write(coordString.encode())

    print("Sent Coordinates: ", coordString)




