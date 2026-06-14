import serial
from queue import Queue
import time
from threading import Thread


eyeSer = serial.Serial('COM5', 115200)
eyeSer.timeout = 0.02
eyeString = Queue(maxsize = 30)

def readSerial():
    while True:
        if eyeSer.readable:
            curString = eyeSer.readline().decode('utf-8').strip()
            #Should have a check here for if it's empty
            print("eyeSerial: " + curString) #print the serial stuffs


def printText(text):
    print(text)
    text = text.encode("utf-8")
    eyeSer.write(text)
    time.sleep(1)



def main():
    serialThread = Thread(target = readSerial)

    while (1):
        printText("pointEye 10, 10, 0")
        printText("pointEye 10, -10, 0")
        printText("pointEye -10, -10, 0")
        printText("pointEye -10, 10, 0")

if __name__ == "__main__":
    main()
