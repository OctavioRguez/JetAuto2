#!/usr/bin/python3
import numpy as np

class OmnidirectionalControl:
    def __init__(self) -> None:
        Tf = 10
        t = 0
        dt = 0.01
        x = 0
        y = 0
        theta = 0

        kpr = 1
        kpt = 10
        vmax = 2

        xd = 5
        yd = 5
        i = 1

        Theta = []
        Thetae = []
        Time = []
        RobotPath = []

while t < Tf:
    thetad = np.arctan2(yd - y, xd - x)
    thetae = theta - thetad

    if thetae > np.pi:
        thetae -= 2 * np.pi
    if thetae < -np.pi:
        thetae += 2 * np.pi

    w = -kpr * thetae

    d = np.sqrt((xd - x) ** 2 + (yd - y) ** 2)

    v = kpt * d
    if v > vmax:
        v = vmax

    xp = v * np.cos(theta)
    yp = v * np.sin(theta)
    thetap = w
    x += xp * dt
    y += yp * dt
    theta += thetap * dt
    if theta > np.pi:
        theta -= 2 * np.pi
    if theta < -np.pi:
        theta += 2 * np.pi

    Time.append(t)
    RobotPath.append([x, y])

    i += 1
    t += dt

RobotPath = np.array(RobotPath)