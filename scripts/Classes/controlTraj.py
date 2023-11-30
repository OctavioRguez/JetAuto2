#!/usr/bin/python3
import numpy as np

class DifferentialDriveRobot:
    def __init__(self, kpl:float, kpr:float, vmax:float, wmax:float, allowErr:float) -> None:
        self.__kpl = kpl
        self.__kpr = kpr
        self.__vmax = vmax
        self.__wmax = wmax
        self.__errThreshold = allowErr

        self.__angVel = 0.0
        self.__linVel = 0.0
        self.__prevPos = None

    def follow(self, point:tuple, currPos:tuple, currAngle:float) -> bool:
        error = np.linalg.norm(np.array(self.__prevPos) - np.array(currPos)) if self.__prevPos is not None else None
        self.__prevPos = currPos if error is None or error > 0.03 else self.__prevPos
        targetx, targety = point
        x, y = self.__prevPos
        # Calculate control input
        diffX = targetx - x
        diffY = targety - y

        # Set angular velocity
        thetad = np.arctan2(diffY, diffX)
        # Set angular velocity
        thetae = currAngle - thetad
        thetae += 2*np.pi if (thetae < -np.pi) else -2*np.pi if (thetae > np.pi) else 0 # Set the angle between -pi and pi
        self.__angVel = self.__wmax*np.tanh(-self.__kpr*thetae/self.__wmax)

        # Set linear velocity
        dist = np.sqrt((diffX)**2 + (diffY)**2)
        self.__linVel = self.__vmax*np.tanh(self.__kpl*dist/self.__vmax) if abs(thetae) < np.pi/12 else 0.0
        # print("point " + str(point))
        # print("pos " + str(currPos))
        print("thetad " + str(thetad))
        print("thetae " + str(thetae))
        # print("dist " + str(dist))
        return dist < self.__errThreshold and abs(thetae) < self.__errThreshold

    def getLinearVel(self) -> float:
        return self.__linVel
    
    def getAngularVel(self) -> float:
        return self.__angVel
