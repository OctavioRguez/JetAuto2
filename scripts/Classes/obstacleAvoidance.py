#!/usr/bin/python3

# Class to avoid obstacles
class obstacleAvoidance:
    def __init__(self, safeDistance:float) -> None:
        # Count to avoid jerking
        self.__count1, self.__count2 = 0, 0
        # Safe distance from obstacles at any direction (m)
        self.__safeDistance = safeDistance

        # Max and min linear velocity (m/s)
        self.__maxLinear, self.__minLinear = 0.08, 0.0
        # Max and min angular velocity (rad/s)
        self.__maxAngular, self.__minAngular = 0.2, 0.0

        # Linear (x) and angular (z) velocities (m/s, rad/s)
        self.__linear, self.__angular = 0.0, 0.0

    # Protected function for avoiding obstacles by changing linear and angular velocities
    def _avoidObstacles(self, scanData: list) -> None:
        # Minimum distance from obstacles at each direction
        forwardDist = min(scanData[0:144] + scanData[1004:1147])
        rightDist = min(scanData[861:1004])
        leftDist = min(scanData[144:287])
        dists = [forwardDist, leftDist, rightDist]

        # Prioritize right rotation
        if rightDist > 1.3*self.__safeDistance:
            self.__changeVelocity(self.__maxLinear/2, -self.__maxAngular)
        # Check if there are no obstacles at any direction
        elif all(dist > self.__safeDistance for dist in dists) or forwardDist > self.__safeDistance:
            self.__changeVelocity(self.__maxLinear, self.__minAngular)
        # Check if there are obstacles in all directions
        elif all(dist < self.__safeDistance for dist in dists):
            self.__changeVelocity(self.__minLinear, self.__maxAngular)
        else:
            self.__rotate(dists, leftDist, rightDist)

    # Private function for turning right of left
    def __rotate(self, dists:list, left:list, right:list) -> None:
        if left > right:
            self.__count1 += 1
            self.__count2 = 0
            # Turn is taken after 20 counts, to avoid jerking
            if self.__count1 >= 20:
                self.__changeVelocity(self.__minLinear, self.__maxAngular)
                if all(dist > self.__safeDistance for dist in dists):
                    self.__count1 = 0
                    self.__changeVelocity(self.__maxLinear, self.__minAngular)
        elif left < right :
            self.__count1 = 0
            self.__count2 += 1
            # Turn is taken after 20 counts, to avoid jerking
            if self.__count2 >= 20:
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
