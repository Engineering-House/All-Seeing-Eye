import cv2
import time





hog_face_detector = dlib.get_frontal_face_detector()

hogDetectFaces('testImage.png', hog_face_detector, display=True)