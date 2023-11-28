#!/usr/bin/python3
import numpy as np

class DifferentialDriveRobot:
    def __init__(self, kpl:float, kpr:float, vmax:float, wmax:float) -> None:
        self.__kpl = kpl
        self.__kpr = kpr
        self.__vmax = vmax
        self.__wmax = wmax

        self.__angVel = 0.0
        self.__linVel = 0.0

    def follow(self, point:tuple, currPos:tuple, currAngle:float) -> None:
        targetx, targety = point
        x, y = currPos
        # Calculate control input
        diffX = targetx - x
        diffY = targety - y

        # Set angular velocity
        thetad = np.arctan2(diffX, diffY)
        # Set angular velocity
        thetae = currAngle - thetad
        thetae += 2*np.pi if (thetae < -np.pi) else -2*np.pi if (thetae > np.pi) else 0 # Saturated angle
        self.__angVel = self.__wmax*np.tanh(self.__kpr*thetae/self.__wmax)

        # Set linear velocity 
        dist = np.sqrt((diffX)**2 + (diffY)**2)
        self.__linVel = self.__vmax*np.tanh(self.__kpl*dist/self.__vmax)

    def getLinearVel(self) -> float:
        return self.__linVel
    
    def getAngularVel(self) -> float:
        return self.__angVel
