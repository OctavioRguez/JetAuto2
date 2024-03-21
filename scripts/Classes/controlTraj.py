#!/usr/bin/python3
import numpy as np

class DifferentialDriveRobot:
    def __init__(self, kpt:float, vmax:float, allowErr:float, bias:float) -> None:
        # Constants
        self.__kpt = kpt
        self.__vmax = vmax
        self.__bias = bias
        self.__errThreshold = allowErr

        # Velocities
        self.__xVel = 0.0
        self.__yVel = 0.0
        # Previous position
        self.__prevPos = None

    def follow(self, point:tuple, currPos:tuple) -> bool:
        error = np.linalg.norm(np.array(self.__prevPos) - np.array(currPos)) if self.__prevPos is not None else None
        self.__prevPos = currPos if error is None or error > 0.01 else self.__prevPos
        targetx, targety = point
        x, y = currPos
        # Calculate control input
        diffX = targetx - x
        diffY = self.__bias*(targety - y)

        # Set linear velocity
        dist = np.sqrt((diffX)**2 + (diffY)**2)
        self.__xVel = self.__vmax*np.tanh(diffX * self.__kpt/self.__vmax) if abs(diffX) > 0.03 else 0.0
        self.__yVel = self.__vmax*np.tanh(diffY * self.__kpt/self.__vmax) if abs(diffY) > 0.03 else 0.0
        return dist < self.__errThreshold

    def getXVel(self) -> float:
        return self.__xVel
    
    def getYVel(self) -> float:
        return self.__yVel
