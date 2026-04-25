import numpy as np
import cv2 as cv
import math
import serial
import time
from queue import Queue
from threading import Thread
import dlib


#open serial ports
eyeSer = serial.Serial('COM5', 115200)
eyeSer.timeout = 0.02
eyeString = Queue(maxsize = 30)

eyeAngle = [0, 0, 0] #pos of the eye angle
printSerial = True

lastSend = time.time()
printString = ["", "", "", ""]



def readSerial():
    global eyeSer, eyeAngle, printSerial, eyeString
    while True:
        #If there is something in the eye serial buffer, read it, strip it, put it in the eye que and print
        if eyeSer.readable:
            curString = eyeSer.readline().decode('utf-8').strip()
            #print(curString, repr(curString))
            if curString == '':
                continue
            else:
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
                #else:
                    #eyeString.put(curString)

def hogDetectFaces(image, hog_face_detector, display = True):
    '''
    This function performs face(s) detection on an image using dlib hog face detector.
    Args:
        image:             The input image of the person(s) whose face needs to be detected.
        hog_face_detector: The hog face detection model required to perform the detection on the input image.
        display:           A boolean value that is if set to true the function displays the original input image, 
                           and the output image with the bounding boxes drawn and time taken written and returns nothing.
    Returns:
        output_image: A copy of input image with the bounding boxes drawn.
        results:      The output of the face detection process on the input image.
    '''
    
    # Get the height and width of the input image.
    height, width, _ = image.shape
    
    # Create a copy of the input image to draw bounding boxes on.
    output_image = image.copy()
    
    # Convert the image from BGR into RGB format.
    imgRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    # Get the current time before performing face detection.
    start = time.time()

    # Perform the face detection on the image.
    results = hog_face_detector(imgRGB, 0)
    
    # Get the current time after performing face detection.
    end = time.time()

    # Loop through the bounding boxes of each face detected in the image.
    for bbox in results:
        
        # Retrieve the left most x-coordinate of the bounding box.
        x1 = bbox.left()
        
        # Retrieve the top most y-coordinate of the bounding box.
        y1 = bbox.top()
        
        # Retrieve the right most x-coordinate of the bounding box.
        x2 = bbox.right()
        
        # Retrieve the bottom most y-coordinate of the bounding box.       
        y2 = bbox.bottom()

        # Draw a rectangle around a face on the copy of the image using the retrieved coordinates.
        cv.rectangle(output_image, pt1=(x1, y1), pt2=(x2, y2), color=(0, 255, 0), thickness=width//200)
    
    
    # Check if the original input image and the output image are specified to be displayed.
    if display:
        
        # Write the time take by face detection process on the output image. 
        cv.putText(output_image, text='Time taken: '+str(round(end - start, 2))+' Seconds.', org=(10, 65),
                    fontFace=cv2.FONT_HERSHEY_COMPLEX, fontScale=width//700, color=(0,0,255), thickness=width//500)
        
        # Display the original input image and the output image.
        plt.figure(figsize=[15,15])
        plt.subplot(121);plt.imshow(image[:,:,::-1]);plt.title("Original Image");plt.axis('off');
        plt.subplot(122);plt.imshow(output_image[:,:,::-1]);plt.title("Output");plt.axis('off');
        
    # Otherwise
    else:
        
        # Return the output image and results of face detection.
        return output_image, results
    


def main():
    global eyeAngle, printSerial, lastSend, printString

    cap = cv.VideoCapture(1)
    
    #Grab pre-trained face tacking stuffs
    hog_face_detector = cv.data.dlib.get_frontal_face_detector()

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

        laksmd = hogDetectFaces(translated_image, hog_face_detector, display = True)

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
            eyeSer.write(printString[2].encode('utf-8'))
            eyeSer.write(printString[3].encode('utf-8'))
            lastSend = time.time()
            printString[2] = ""
            printString[3] = ""

        #Should be removed when there is no desktop enviroment
        cv.imshow('Face Detection', translated_image)
        if cv.waitKey(25) == ord('q'):
            break

        time.sleep(.1)


    #Realease the camera
    cap.release()
    cv.destroyAllWindows()
    exit

    

if __name__ == "__main__":
    serialThread = Thread(target = readSerial)
    serialThread.start()
    #Call main
    main()