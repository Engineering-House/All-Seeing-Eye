import numpy as np
import cv2 as cv
import math
import serial
import time
from queue import Queue
from threading import Thread


#open serial ports
eyeSer = serial.Serial('COM5', 115200)
eyeSer.timeout = 0.02
eyeString = Queue(maxsize = 30)

eyeAngle = [0, 0, 0] #pos of the eye angle
printSerial = True


def readSerial():
    global eyeSer, eyeAngle, printSerial, eyeString
    while True:
        #If there is something in the eye serial buffer, read it, strip it, put it in the eye que and print
        if eyeSer.readable:
            curString = eyeSer.readline().decode('utf-8').strip()
            #print(curString, repr(curString))
            if curString == '':
                continue

            #Should have a check here for if it's empty
            if printSerial:
                print("eyeSerial: " + curString) #print the serial stuffs

            #Parce data and put info in global vars
            #If not parced properly put in que for other thread
            if curString[:7:] == 'data: ': 
                curString = curString[8:]

                print(repr(curString))

                curString = curString.split(', ')

                if not(curString[curString.len() - 1]) == 0:
                    print("error number: ")
                    print(curString[curString.len() - 1])
                
                eyeAngle = [curString[0], curString[1], curString[2]]
            else:
                eyeString.put(curString)




def main():
    global eyeAngle, printSerial

    cap = cv.VideoCapture(1)
    
    #Grab pre-trained face tacking stuffs
    face_classifier = cv.CascadeClassifier(
        cv.data.haarcascades + "haarcascade_frontalface_default.xml"
    )

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

    
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        #Rotate the image
        height, width = frame.shape[:2]
        center = (width // 2, height // 2)

        #set perameters
        theta = 0
        scale = 1

        #Setup and perform the rotation
        rotation_matrix = cv.getRotationMatrix2D(center, theta, scale)
        translated_image = cv.warpAffine(frame, rotation_matrix, (width, height))

        #Look for faces and save property
        face = face_classifier.detectMultiScale(translated_image, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40))

        faces = []
        i = 0
        # Draw rectangles around detected faces
        for (x, y, w, h) in face:
            faces.append([x.item(), y.item(), w.item(), h.item()])
            i += 1
            cv.rectangle(translated_image, (x, y), (x + w, y + h), (0, 255, 0), 4)

        #If a face is detected
        if len(faces) > 0:
            largestFace = 0
            i = 0
            for i in range(len(faces)):
                if (faces[largestFace][2] * faces[largestFace][2]) < (faces[i][2] * faces[i][2]):
                    largestFace = i

            distance = (width * height) / (faces[largestFace][2] * faces[largestFace][3])
            centerX = faces[largestFace][0] - .5*(width - faces[largestFace][2])
            centerY = faces[largestFace][1] - .5*(height - faces[largestFace][3])

            height, width = translated_image.shape[:2]
            x = 180 * centerX // (width//2) + 90
            y = 180 * centerY // (height//2) + 90

            print(x, y)

            info = "pointEye" + str(x) + ", " + str(y) + ", " + str(distance)
            info = info.encode('utf-8')
            eyeSer.write(info)


        #Should be removed when there is no desktop enviroment
        cv.imshow('Face Detection', translated_image)
        if cv.waitKey(25) == ord('q'):
            break

        time.sleep(.1)


    #Realease the camera
    cap.release()
    cv.destroyAllWindows()

    

if __name__ == "__main__":
    serialThread = Thread(target = readSerial)
    serialThread.start()
    #Call main
    main()