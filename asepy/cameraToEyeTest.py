import numpy as np
import cv2 as cv
import math
import serial
import time
from queue import Queue
from threading import Thread
from ultralytics import YOLO

#open serial ports
# eyeSer = serial.Serial('COM5', 115200)
# eyeSer.timeout = 0.02
eyeString = Queue(maxsize = 30)

eyeAngle = [0, 0, 0] #pos of the eye angle
printSerial = True

lastSend = time.time()
printString = ["", "", "", ""]

yoloModel = YOLO("yolo11n.pt")

# def readSerial():
#     global eyeSer, eyeAngle, printSerial, eyeString
#     while True:
#         #If there is something in the eye serial buffer, read it, strip it, put it in the eye que and print
#         if eyeSer.readable:
#             curString = eyeSer.readline().decode('utf-8').strip()
#             #print(curString, repr(curString))
#             if curString == '':
#                 continue
#             else:
#                 #Should have a check here for if it's empty
#                 if printSerial:
#                     print("eyeSerial: " + curString) #print the serial stuffs

#                 #Parce data and put info in global vars
#                 #If not parced properly put in que for other thread
#                 if curString[:7:] == 'data: ': 
#                     curString = curString[8:]

#                     print(repr(curString))

#                     curString = curString.split(', ')

#                     if not(curString[curString.len() - 1]) == 0:
#                         print("error number: ")
#                         print(curString[curString.len() - 1])
                    
#                     eyeAngle = [curString[0], curString[1], curString[2]]
#                 #else:
#                     #eyeString.put(curString)

# consumes img and uses it to draw on if displaying
def yoloDetectFaces(img, display = True, confMin = 0.5):
    results = yoloModel(img)
    out = []
    if(len(results) > 0):
        result = results[0]
        for i in range(len(result.boxes.xyxy)):
            xyxy = result.boxes.xyxy[i]
            if(result.names[result.boxes.cls.int()[i].item()] == "person" and
               result.boxes.conf[i] > confMin):
                out += [[xyxy[0].item(), xyxy[1].item(), xyxy[2].item(), xyxy[3].item()]]
                if(display):
                    pos1 = (int(xyxy[0].item()), int(xyxy[1].item()))
                    pos2 = (int(xyxy[2].item()), int(xyxy[3].item()))
                    # cv.rectangle(frame, (xyxy[0].item(), xyxy[1].item()), (xyxy[2].item(), xyxy[3].item()), (0,255,0), 3)
                    cv.rectangle(img, pos1, pos2, (0,255,0), 3)
        if(display):
            cv.imshow("meow", img)
    return img, out


def main():
    global eyeAngle, printSerial, lastSend, printString

    cap = cv.VideoCapture(0)

    #Check if the camera is attached
    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    while True:

        #Capture frame by frame
        ret, frame = cap.read()

        #if frame is read correctly ret is true
        if not ret:
            print("Can't receive video, exiting")
            break

        #Rotate the image
        height, width = frame.shape[:2]
        center = (width // 2, height // 2)

        imgout, faces = yoloDetectFaces(frame, True, 0.7)

        #If a face is detected
        if len(faces) > 0:
            largestFace = 0
            i = 0
            for i in range(len(faces)):
                if (faces[largestFace][2] * faces[largestFace][2]) < (faces[i][2] * faces[i][2]):
                    largestFace = i

            distance = (width * height) / (faces[largestFace][2] * faces[largestFace][3])
            centerX = faces[largestFace][0] + .5*faces[largestFace][2]
            #centerX = faces[largestFace][0] - .5*(width - faces[largestFace][2])
            centerY = faces[largestFace][1] + .5*faces[largestFace][3]
            #centerY = faces[largestFace][1] - .5*(height - faces[largestFace][3])

            x = int(centerX - width//2)
            y = int(-1*(centerY - height//2))

            maxDeflection = 55

            if (not(x == 0) and not(y == 0) and not(distance == 0)):
                trueDistance = math.sqrt(x*x + y*y + distance*distance)
                unitVector = [x/trueDistance, y/trueDistance, distance/trueDistance]
                
            
                mtr2Pos = math.pi/2 - math.acos(unitVector[0]) #Angle of unit vecotr in Z X plane
                mtr3Pos = math.pi/2 - math.acos(unitVector[1]) #Angel of unit vector in Z Y plane

                #covert to deg
                mtr2Pos = mtr2Pos * (180/math.pi)
                mtr3Pos = mtr3Pos * (180/math.pi)
            else:
                mtr2Pos = 0
                mtr3Pos = 0

            mtr2Pos = mtr2Pos + 90
            mtr3Pos = mtr3Pos + 90

            if (mtr2Pos < maxDeflection):
                mtr2Pos = maxDeflection
            if (mtr3Pos < maxDeflection):
                mtr3Pos = maxDeflection

            printString[2] = "moveMotr 2 " + str(int(mtr2Pos))
            printString[3] = "moveMotr 3 " + str(int(mtr3Pos))


        if (time.time() > lastSend + .1 and printString[2] != "" and printString[3] != ""):
            print(printString[2], printString[3])
            # eyeSer.write(printString[2].encode('utf-8'))
            # eyeSer.write(printString[3].encode('utf-8'))
            lastSend = time.time()
            printString[2] = ""
            printString[3] = ""

        #Should be removed when there is no desktop enviroment
        cv.imshow('Face Detection', imgout)
        if cv.waitKey(1) == ord('q'):
            break


    #Realease the camera
    cap.release()
    cv.destroyAllWindows()
    exit()

    

if __name__ == "__main__":
    # serialThread = Thread(target = readSerial)
    # serialThread.start()
    #Call main
    main()