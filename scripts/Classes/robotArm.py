#!/usr/bin/python3
import numpy as np

# Class for managing the robotic arm 
class robotArm:
    def __init__(self, ls:list) -> None:
        # Initialize each joint
        self.__q = {"q1":None, "q2":None, "q3":None, "q4":None}
        # Initialize the length of each link
        self.__l = {"l1":ls[0], "l2":ls[1], "l3":ls[2], "l4":ls[3]}

    # Private function for solving the inverse kinematics
    def _inverseKinematics(self, x:float, y:float, z:float) -> None:
        # Check if the arm is not going to hit
        if x >= 0.0 and z > -0.16 and (-0.15 > y > 0.15 or x > 0.17 or z > 0.05):
            Pwx = np.sqrt(x**2 + y**2) - self.__l["l4"]
            Pwy = z - self.__l["l1"]
            alpha = np.arctan2(Pwy, Pwx)
            s = np.sqrt(Pwx**2 + Pwy**2)

            D = (s**2 - self.__l["l2"]**2 - self.__l["l3"]**2)/(2*self.__l["l2"]*self.__l["l3"])
            # Check if the point is reachable
            if abs(D) < 1.0:
                # Calculate the joint angles
                self.__q["q1"] = np.arctan2(y, x)
                self.__q["q3"] = -np.arctan2(np.sqrt(1 - D**2), D)

                gamma = np.arctan2(self.__l["l3"]*np.sin(self.__q["q3"]), self.__l["l2"] + self.__l["l3"]*np.cos(self.__q["q3"]))
                self.__q["q2"] = alpha - gamma
                self.__q["q4"] = -(self.__q["q2"] + self.__q["q3"])
                return
        for qn in self.__q.keys():
            self.__q[qn] = None

    # Private function for getting the final trajectory
    def _trajectory(self, qIni:list, qFin:list, rate:int) -> list:
        if any(q is None for q in qFin):
            return np.array([])
        return np.linspace(qIni, qFin, rate)

    # Private function for starting the joints
    def _startArm(self) -> list:
        self.__q["q1"] = 0.0
        self.__q["q2"] = -np.pi/4
        self.__q["q3"] = 5*np.pi/8
        self.__q["q4"] = np.pi/4
        return list(self.__q.values())

    # Private function for resetting the joints
    def _resetArm(self) -> list:
        self.__q["q1"] = 0.0
        self.__q["q2"] = -np.pi/4
        self.__q["q3"] = 5*np.pi/8
        self.__q["q4"] = 3*np.pi/8
        return list(self.__q.values())

    # Public function for getting the joints
    def getJoints(self) -> list:
        if any(q is None for q in self.__q.values()):
            return None
        self.__q["q2"] = np.pi/2 - self.__q["q2"] # Add pi/2 offset for the joint 2 angle
        self.__q["q3"] *= (-1) # Invert the joint 3 angle
        self.__q["q4"] *= (-1) # Invert the joint 4 angle
        return list(self.__q.values())
