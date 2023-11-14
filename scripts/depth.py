#!/usr/bin/python3
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image, CameraInfo

class DepthNode:
    def __init__(self) -> None:
        self.__matrixCam = None
        rospy.Subscriber("/camera/depth/image", Image, self.__imageCallback)
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.__infoCallback)

    def __imageCallback(self, msg:Image) -> None:
        img = cv.imdecode(np.frombuffer(msg.data, np.uint16), cv.IMREAD_COLOR)
        if self.__matrixCam is not None:
            self._reproject(img)

    def __infoCallback(self, msg:CameraInfo) -> None:
        self.__matrixCam = np.asfarray(msg.K).reshape(3,3)

    def _reproject(self, img:np.ndarray) -> np.ndarray:
        height, width = img.shape
        for row in range(height):
            for col in range(width):
                depth = img[row, col]
                meters = self.__validDepth(depth)

    def __validDepth(self, depth:np.uint16) -> float:
        meters = depth * 0.001
        if meters < 0.3 or meters > 3.0:
            return None
        return meters

    def _stop(self) -> None:
        print("Stopping depth node")

if __name__ == '__main__':
    rospy.init_node("DepthNode")
    rate = rospy.Rate(15)
    nodeHandler = DepthNode()
    rospy.on_shutdown(nodeHandler._stop)

    print("The DepthNode is Running")
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except rospy.ROSInterruptException as ie:
            rospy.loginfo(ie) # Catch an Interruption
