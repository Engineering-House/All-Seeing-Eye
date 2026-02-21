import numpy as np
import cv2 as cv
import math
import serial

eyeAngle = [0, 0, 0]

#open serial port
ser = serial.Serial('COM5', 115200)
ser.timeout = 0.02
print(ser.is_open)

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
    #If the serial is reabale, read it 
    #Okok so what seems to be happeneing is that it this there is stuff to read
    #Then there isn't so it times out n fucks everything up
    #This should be asinc and not blocking
    #thinking I should create a thread that looks at the serial
    #if there is a line then put it into a que
    #then if the que is populated then pop something off
    #read and do stuff
    #and then we win!
    #also should allow serial inputs from python therminal to be copied onto micro controller
    if ser.readable:
        curString = "                                          "#ser.readline().decode('utf-8').strip()
        print(curString)

        #If it is data, parce and set eye angle
        #Check error number
        if curString[:7:] == 'data: ': 
            curString = curString[8:]

            print(repr(curString))

            curString = curString.split(', ')

            if not(curString[curString.len() - 1]) == 0:
                print("error number: ")
                print(curString[curString.len() - 1])
            
            eyeAngle = [curString[0], curString[1], curString[2]]


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



cap.release()
cv.destroyAllWindows()


