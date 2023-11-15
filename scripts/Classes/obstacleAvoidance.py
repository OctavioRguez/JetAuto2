#!/usr/bin/python3

# Class to avoid obstacles
class obstacleAvoidance:
    def __init__(self, safeDistance:float) -> None:
        # Count to avoid jerking
        self.__count1, self.__count2 = 0, 0
        # Safe distance from obstacles at any direction (m)
        self.__safeDistance = safeDistance

        # Max and min linear velocity (m/s)
        self.__maxLinear, self.__minLinear = 0.15, 0.0
        # Max and min angular velocity (rad/s)
        self.__maxAngular, self.__minAngular = 0.5, 0.0

        # Linear (x) and angular (z) velocities (m/s, rad/s)
        self.__linear, self.__angular = 0.0, 0.0

    # Private function for avoiding obstacles by changing linear and angular velocities
    def _avoidObstacles(self, scanData: list) -> None:
        # Minimum distance from obstacles at each direction
        forwardDist = min(scanData[:144] + scanData[1004:1147])
        rightDist = min(scanData[41:180])
        leftDist = min(scanData[540:680])
        dists = [forwardDist, leftDist, rightDist]

        # Check if there are no obstacles at any direction
        if all(dist > self.__safeDistance for dist in dists):
            self.__changeVelocity(self.__maxLinear, self.__minAngular)
        # Check if there are obstacles in all directions
        elif all(dist < self.__safeDistance for dist in dists):
            self.__changeVelocity(self.__minLinear, -self.__maxAngular)
        else:
            if leftDist > rightDist:
                self.__count1 += 1
                self.__count2 = 0
                # Turn is taken after 5 counts, to avoid jerking
                if self.__count1 >= 5:
                    self.__changeVelocity(self.__minLinear, self.__maxAngular)
                    if all(dist > self.__safeDistance for dist in dists):
                        self.__count1 = 0
                        self.__changeVelocity(self.__maxLinear, self.__minAngular)

            elif leftDist < rightDist :
                self.__count1 = 0
                self.__count2 += 1
                # Turn is taken after 5 counts, to avoid jerking
                if self.__count2 >= 5:
                    self.__changeVelocity(self.__minLinear, -self.__maxAngular)
                    if all(dist > self.__safeDistance for dist in dists):
                        self.__count2 = 0
                        self.__changeVelocity(self.__maxLinear, self.__minAngular)

    # Private function for changing linear and angular velocities
    def __changeVelocity(self, linear:float, angular:float) -> None:
        self.__linear = linear
        self.__angular = angular

    # Public function for changing linear velocity limits
    def changeLinearVelocity(self, min:float, max:float) -> None:
        self.__minLinear, self.__maxLinear = min, max

    # Public function for changing angular velocity limits
    def changeAngularVelocity(self, min:float, max:float) -> None:
        self.__minAngular, self.__maxAngular = min, max

    # Public funtion for getting linear velocity
    def getLinear(self) -> float:
        return self.__linear
    
    # Public funtion for getting angular velocity
    def getAngular(self) -> float:
        return self.__angular