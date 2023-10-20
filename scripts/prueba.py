#!/usr/bin/python3
import cv2 as cv
from Classes.modelPredict import modelPredict

def stop() -> None:
    cap.release()
    print("Stopping")
    exit(1)

if __name__ == '__main__':
    # Classificator class object
    classes = ["Fanta", "Pepsi", "Seven"]
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    model = modelPredict("../Model/bestV5-20e.onnx", classes, colors, 640, 0.6, True)

    cap = cv.VideoCapture(0)

    while True:
        try:
            if cap.isOpened():
                ret, frame = cap.read() # Get actual frame
                if not ret:
                    print("Unable to obtain current frame")
            else:
                print("Error: Could not open camera")
            model._startDetection(frame, "Fanta", 0.1)

            if cv.waitKey(1) == ord('q'):
                stop()
        except KeyboardInterrupt as ki:
            print(ki)
            stop()