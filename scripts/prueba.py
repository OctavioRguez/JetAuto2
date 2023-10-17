#!/usr/bin/python3
import cv2 as cv
from Classes.modelPredict import modelPredict

def stop() -> None:
    cap.release()
    print("Stopping")

if __name__ == '__main__':
    # Classificator class object
    model = modelPredict("../Model/bestV4-40e.onnx", ["Fanta", "Pepsi", "Seven"], [(255, 0, 0), (0, 255, 0), (0, 0, 255)])

    cap = cv.VideoCapture(0)

    while True:
        try:
            if cap.isOpened():
                ret, frame = cap.read() # Get actual frame
                if not ret:
                    print("Unable to obtain current frame")
            else:
                print("Error: Could not open camera")
            model._startDetection("Fanta", frame, 0.1)
        except KeyboardInterrupt as ki:
            print(ki)
            stop()