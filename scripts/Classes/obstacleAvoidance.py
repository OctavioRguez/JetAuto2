#!/usr/bin/python3

# Class to avoid obstacles
class obstacleAvoidance:
    def __init__(self, safeDistance:float) -> None:
        # Count to avoid jerking
        self.__count1, self.__count2 = 0, 0
        # Safe distance from obstacles at any direction (m)
        self.__safeDistance = safeDistance

        # Linear (x) and angular (z) velocities (m/s, rad/s)
        self.__linear = 0.0
        self.__angular = 0.0

    # Private function for avoiding obstacles by changing linear and angular velocities
    def _avoidObstacles(self, scanData: list) -> None:
        # Minimum distance from obstacles at each direction
        forwardDist = min(scanData[:41] + scanData[680:719])
        rightDist = min(scanData[41:180])
        leftDist = min(scanData[540:680])
        dists = [forwardDist, leftDist, rightDist]

        # Check if there are no obstacles at any direction
        if all(dist > self.__safeDistance for dist in dists):
            self.__linear = 0.15
            self.__angular = 0.0
        # Check if there are obstacles in all directions
        elif all(dist < self.__safeDistance for dist in dists):
            self.__linear = 0
            self.__angular = -0.5
        else:
            if leftDist > rightDist:
                self.__count1 += 1
                self.__count2 = 0
                # Turn is taken after 5 counts, to avoid jerking
                if self.__count1 >= 5:
                    self.__linear = 0
                    self.__angular = 0.5

                    if all(dist > self.__safeDistance for dist in dists):
                        self.__count1 = 0
                        self.__linear = 0.15
                        self.__angular = 0
            elif leftDist < rightDist :
                self.__count1 = 0
                self.__count2 += 1
                # Turn is taken after 5 counts, to avoid jerking
                if self.__count2 >= 5:
                    self.__linear = 0
                    self.__angular = -0.5
        
                    if all(dist > self.__safeDistance for dist in dists):
                        self.__count2 = 0
                        self.__linear = 0.15
                        self.__angular = 0

    # Public funtion for getting linear velocity
    def getLinear(self) -> float:
        return self.__linear
    
    # Public funtion for getting angular velocity
    def getAngular(self) -> float:
        return self.__angular