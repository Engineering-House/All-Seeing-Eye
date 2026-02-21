from inputs import devices, get_gamepad
import math
import serial
import time
from threading import Thread

if len(devices.gamepads) < 1:
    print("Gamepad not found")
else:
    print(f"Gamepad \"{devices.gamepads[0]}\" found")

gamepad = devices.gamepads[0]

lx = 0
ly = 0

lastSend = time.time()
lastInput = time.time()

def gamepad_input():
    global lx, ly, lastInput
    while True:
        events = get_gamepad()
        for event in events:
            lastInput = time.time()
            if event.code == "ABS_X":
                lx = event.state / (2**15)
            if event.code == "ABS_Y":
                ly = event.state / (2**15)

def main():
    global lx, ly
    lastSend = time.time()
    try:
        ser = serial.Serial(
            port="/dev/ttyUSB0",
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=2
        )
        ser.flushInput()
        ser.flushOutput()

        #time.sleep(2)

        print("Connected to /dev/ttyUSB0 at 9600 baud")
    except serial.SerialException as e:
        print(f"Failed to connect: {e}")
        exit(1)
    while True:
        if time.time() > lastInput + 5:
            angle = 3.14159/2
            mag = math.sin(time.time()) * 0.4
        else:
            angle = math.atan2(ly, -lx)
            mag = math.sqrt(lx * lx + ly * ly)
        if time.time() > lastSend + 0.1:
            lastSend = time.time()
            print(f"Angle: {angle} Magnitude: {mag}")
            mot0 = math.cos(angle) * mag * 6000
            mot1 = math.cos(angle + 2 * 3.14259 / 3) * mag * 6000
            mot2 = -math.cos(angle - 2 * 3.14159 / 3) * mag * 6000

            ser.flushInput()
            ser.flushOutput()

            smot0 = bytes([ord(char) for char in f"pos 0 {int(mot0)}\r\n"])
            smot1 = bytes([ord(char) for char in f"pos 1 {int(mot1)}\r\n"])
            smot2 = bytes([ord(char) for char in f"pos 2 {int(mot2)}\r\n"])
            ser.write(smot0)
            ser.readline()
            ser.readline()
            ser.write(smot1)
            ser.readline()
            ser.readline()
            ser.write(smot2)
            ser.readline()
            ser.readline()
        #ser.read(ser.in_waiting)
        #ser.write(f"pos 0 {int(mot0)}\r\n".encode("utf-8"))
        #ser.write(f"pos 1 {int(mot1)}\r\n".encode("utf-8"))
        #ser.write(f"pos 2 {int(mot2)}\r\n".encode("utf-8"))

        #print(f"pos 0 {int(mot0)}")
        #print(f"pos 1 {int(mot1)}")
        #print(f"pos 2 {int(mot2)}")


if __name__ == "__main__":
    thread = Thread(target = gamepad_input)
    thread.start()
    main()
