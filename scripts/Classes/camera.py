#!/usr/bin/python3
import cv2 as cv

class Camera:
    # Variables
    def __init__(self, port:int) -> None:
        self.__cam = cv.VideoCapture(port)

    # Function for getting the current frame
    def getImage(self) -> cv.Mat:
        if self.__cam.isOpened():
            ret, frame = self.__cam.read() # Get actual frame
            if ret:
                return frame
            else:
                print("Unable to obtain current frame")
        else:
            print("Error: Could not open camera")
        return None

    # Function for releasing the camera
    def _releaseCamera(self) -> None:
        self.__cam.release()
        print("Stopping") # Stop message
