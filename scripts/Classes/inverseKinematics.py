#!/usr/bin/python3
import numpy as np

class inverseKinematics:
    def __init__(self, l1:float, l2:float, l3:float, l4:float) -> None:
        # Initialize each joint
        self.__q = {"q1":0.0, "q2":0.0, "q3":0.0, "q4":0.0}

        # Initialize the length of each link
        self.__l = {"l1":l1, "l2":l2, "l3":l3, "l4":l4}

    # Implement inverse kinematics calculation
    def _inverseKinematics(self, x:float, y:float, z:float) -> None:
        Pwx = np.sqrt(x**2 + y**2) - self.__l["l4"]
        Pwy = z - self.__l["l1"]
        
        alpha = np.arctan2(Pwy, Pwx)
        s = np.sqrt(Pwx**2 + Pwy**2)
        D = (s**2 - self.__l["l2"]**2 - self.__l["l3"]**2)/(2*self.__l["l2"]*self.__l["l3"])
        
        self.__q["q1"] = np.arctan2(x, y)
        self.__q["q3"] = np.arctan2(np.sqrt(1 - D**2), D)

        sigma = np.arctan2(self.__l["l3"]*np.sin(self.__q["q3"]), self.__l["l2"] + self.__l["l3"]*np.cos(self.__q["q3"]))
        self.__q["q2"] = alpha - sigma
        self.__q["q4"] = - (self.__q["q2"] + self.__q["q3"])

    # Public function for getting the joints
    def getJoints(self) -> list:
        return self.__q.values()
