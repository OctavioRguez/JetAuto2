#!/usr/bin/python3
import cv2 as cv
from Classes.modelPredict import modelPredict

def stop() -> None:
    cap.release()
    print("Stopping")
    exit(1)

if __name__ == '__main__':
    # Classificator class object
    model = modelPredict("../Model/bestV5-25e.onnx", ["Fanta", "Pepsi", "Seven"], 0.8, False)
    cap = cv.VideoCapture(0)
    while True:
        try:
            if cap.isOpened():
                ret, frame = cap.read() # Get actual frame
                if not ret:
                    print("Unable to obtain current frame")
                elif frame.size > 0:
                    model._startDetection(frame, "Fanta", 0.052)
            else:
                print("Error: Could not open camera")

            if cv.waitKey(1) == ord('q'):
                stop()
        except KeyboardInterrupt as ki:
            print(ki)
            stop()
