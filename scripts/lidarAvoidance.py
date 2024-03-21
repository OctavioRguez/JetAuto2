#!/usr/bin/python3
import rospy
import numpy as np
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
import sys
sys.path.append("/home/tavo/Ciberfisicos_ws/src/jet_auto2/scripts")
from Classes.obstacleAvoidance import obstacleAvoidance

# Avoid obstacles based on LiDAR data
class lidarAvoidance:
    def __init__(self, dist:float) -> None:
        # Create the obstacle avoidance instance
        self.__obstacleManager = obstacleAvoidance(dist)
        # Velocity message
        self.__velocity = Twist()
        # Flag to activate the navigation
        self.__navegate = True

        # Horizontal coordinate of the object
        self.__horizontal = 0.0
        # Control constants
        self.__kp = 1.1
        self.__wmax = 0.35

        # Wait for all the other nodes to be active
        # rospy.wait_for_message("/usb_cam/model_prediction/compressed", CompressedImage, 10)
        rospy.wait_for_message("/map", OccupancyGrid, 20)

        # Initialize the subscribers and publishers
        self.__vel_pub = rospy.Publisher("/jetauto_controller/cmd_vel", Twist, queue_size = 10)
        rospy.Subscriber("/scan", LaserScan, self.__scanCallback)
        rospy.Subscriber("/return", Bool, self.__returnCallback)
        rospy.Subscriber("/object/depth", Float64, self.__depthCallback)
        rospy.Subscriber("/object/horizontal", Float64, self.__horizontalCallback)

    # Callback function for getting the lidar data
    def __scanCallback(self, data:LaserScan) -> None:
        if self.__navegate is True:
            # Check for obstacles
            self.__obstacleManager._avoidObstacles(data.ranges)
            # Get linear and angular velocities
            self.__velocity.linear.x = self.__obstacleManager.getLinear()
            self.__velocity.angular.z = self.__obstacleManager.getAngular()
            self.__vel_pub.publish(self.__velocity) # Publish the velocity

    # Callback function for the object horizontal coordinate
    def __horizontalCallback(self, msg:Float64) -> None:
        self.__horizontal = msg.data

    # Callback function for the object coordinates
    def __depthCallback(self, msg:Float64) -> None:
        # Control
        lin = 0.0 if (msg.data < 0.15) else 0.035 if (msg.data < 0.23) else 0.07
        ang = self.__wmax*np.tanh(-self.__horizontal*self.__kp/self.__wmax)

        # Publish the velocity
        self.__navegate = False
        self.__velocity.linear.x, self.__velocity.angular.z = lin, ang
        self.__vel_pub.publish(self.__velocity)

    # Callback function for the return state
    def __returnCallback(self, msg:Bool) -> None:
        if not msg.data:
            self.__navegate = True # Activate the navigation

    # Stop function
    def _stop(self) -> None:
        print("Stopping the obstacle avoidance")
        self.__velocity.linear.x, self.__velocity.angular.z = 0.0, 0.0 # Stop the robot
        self.__vel_pub.publish(self.__velocity) # Publish the velocity
 
if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("Lidar_Obstacle_Avoidance")

    # Initialize the rate
    rate = rospy.Rate(rospy.get_param("rateObst", default = 15))

    # Get parameters
    safeDist = rospy.get_param("safe_distance/value", default = 0.25)

    # Create the instance of the class
    avoider = lidarAvoidance(safeDist)

    # Shutdown hook
    rospy.on_shutdown(avoider._stop)

    # Run the node
    print("The Obstacle Avoider is Running")
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except rospy.ROSInterruptException as ie:
            rospy.loginfo(ie) # Catch an Interruption
