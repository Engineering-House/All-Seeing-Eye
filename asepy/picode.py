import numpy as np
import cv2 as cv
import math
import serial
import time
from threading import Thread
from queue import Queue
from inputs import devices, get_gamepad

#General global variabls
printSerial = True #if true will print info from both serials
eyeAngle = [0, 0, 0] #pos of the eye angle

#open serial ports
eyeSer = serial.Serial('COM5', 115200)
eyeSer.timeout = 0.02
eyeString = Queue(maxsize = 30)

tenticalSer = serial.Serial('COM4', 115200)
tenticalString = Queue(maxsize = 30)

parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
bytesize=serial.EIGHTBITS,

tenticalSer.flushInput()
tenticalSer.flushOutput()
eyeSer.flushInputs()
eyeSer.flushOutputs()


#Gamepad setup
if len(devices.gamepads) < 1:
    print("Gamepad not found")
else:
    print(f"Gamepad \"{devices.gamepads[0]}\" found")

gamepad = devices.gamepads[0]

lx = 0
ly = 0

lastSend = time.time()
lastInput = time.time()

def gamepadInput():
    global lx, ly, lastInput
    while True:
        events = get_gamepad()
        for event in events:
            lastInput = time.time()
            if event.code == "ABS_X":
                lx = event.state / (2**15)
            if event.code == "ABS_Y":
                ly = event.state / (2**15)


def readSerial():
    global tenticalSer, eyeSer, eyeAngle, printSerial, eyeString, tenticalString
    while True:
        #If there is something in the eye serial buffer, read it, strip it, put it in the eye que and print
        if eyeSer.readable:
            curString = eyeSer.readline().decode('utf-8').strip()
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


        if tenticalSer.readable:
            curString = tenticalSer.readline().decode('utf-8').strip()
            tenticalSer(curString)
            if printSerial:
                print("tenticalSerial: " + curString)
        


def main():
    global tenticalSer, eyeSer, eyeAngle, printSerial, eyeString, tenticalString, lx, ly
    lastSend = time.time()
    

    #grab capture and save to cap
    cap = cv.VideoCapture(0)

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
        theta = 45
        scale = 1

        #Setup and perform the rotation
        rotation_matrix = cv.getRotationMatrix2D(center, theta, scale)
        translated_image = cv.warpAffine(frame, rotation_matrix, (width, height))

        #Look for faces and save property
        face = face_classifier.detectMultiScale(translated_image, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40))

        # Draw rectangles around detected faces
        for (x, y, w, h) in face:
            cv.rectangle(translated_image, (x, y), (x + w, y + h), (0, 255, 0), 4)

        cv.imshow('Face Detection', translated_image)

        if cv.waitKey(25) == ord('q'):
            break


    #Realease the camera
    cap.release()
    cv.destroyAllWindows()

    


if __name__ == "__main__":
    #Start thread to read inputs from serial
    serialThread = Thread(target = readSerial)
    serialThread.start()
    gamepadThread = Thread(target = gamepadInput)
    gamepadThread.start()
    #Call main
    main()