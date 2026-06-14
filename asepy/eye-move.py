import serial
import time
import random
from ultralytics import YOLO
import cv2 as cv

eyeSer = serial.Serial('/dev/ttyUSB1', 115200)
eyeSer.timeout = 0.02

random.seed(5)

yoloModel = YOLO("yolo11n.pt")

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
    cap = cv.VideoCapture(1)

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    x = 0.5
    y = 0.5

    while True:
        ret, img = cap.read()
        if(not ret):
            continue
        print(img.shape)
        # res = cv.resize(img, dsize=(640, 640), interpolation=cv.INTER_CUBIC)
        rotation_matrix = cv.getRotationMatrix2D((360, 640), -60, 1)
        rotated_image = cv.warpAffine(img, rotation_matrix, (1280, 1280))
        imgout, poses = yoloDetectFaces(rotated_image, True, 0.7)


        if(len(poses) > 0):
            best = poses[0]
            for pose in poses:
                if(pose[2] - pose[0] > best[2] - best[0]):
                    best = pose
            print(best)
            cx = (best[0] + best[2]) / 2 / 1280
            cy = (best[1] + best[3]) / 2 / 1280

            if(cx > 0.6 and x > 0):
                x -= 0.1
            if(cx < 0.6 and x < 1):
                x += 0.1

            if(cy > 0.5 and y > 0):
                y -= 0.1
            if(cy < 0.5 and y < 1):
                y += 0.1

            pos = int(x * (180 - 55) + 55)
            # pos1 = int(random.random() * (180 - 55) + 55)
            eyeSer.write(f"moveMotr 2 {pos}\r\n".encode("utf-8"))
        # eyeSer.write(f"moveMotr 3 {pos1}\r\n".encode("utf-8"))
        if cv.waitKey(1) == ord('q'):
            break
        # time.sleep(0.1)

    cap.release()
    cv.destroyAllWindows()
    exit()

main()