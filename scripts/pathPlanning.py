#!/usr/bin/python3
import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append("/home/tavo/Ciberfisicos_ws/src/jet_auto2/scripts")
from Classes.prmAstar import PRMAstar
from Classes.controlTraj import DifferentialDriveRobot

# Avoid obstacles based on LiDAR data
class pathPlanning:
    def __init__(self, points:int, dist:float, density:int) -> None:
        # Create the obstacle avoidance instance
        self.__planningManager = PRMAstar(points, dist, density)
        # Map with obstacles from the SLAM
        self.__mapData = None
        # Position estimated with odometry
        self.__pose = None

        # Create the control manager
        self.__controlManager = DifferentialDriveRobot(0.1, 0.1, 0.1, 0.1)
        self.__velocity = Twist()
        self.__path = np.array([])
        
        # Initialize the subscribers and publishers
        rospy.Subscriber("/map", OccupancyGrid, self.__mapCallback)
        rospy.Subscriber("/odom", Odometry, self.__odomCallback)
        rospy.Subscriber("/initiate", Bool, self.__initCallback)
        self.__vel_pub = rospy.Publisher("/jetauto_controller/cmd_vel", Twist, queue_size=10)

    # Callback function for getting the lidar data
    def __mapCallback(self, msg:OccupancyGrid) -> None:
        self.__mapData = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        # Plot map
        plt.figure()
        plt.imshow(self.__mapData, cmap='Greys', origin='lower')
        plt.savefig("/home/tavo/map.png")

    # Callback function for getting the current odometry position
    def __odomCallback(self, msg:Odometry) -> None:
        self.__pose = msg.pose.pose.position
        rospy.loginfo(self.__pose)

    # Callback function for getting the flag for starting the path planning
    def __initCallback(self, msg:Bool) -> None:
        if msg.data is True:
            self.__planning()

    # Start function
    def __planning(self) -> None:
        if self.__mapData is not None and self.__pose is not None:
            w, h = self.__mapData.shape
            x, y = self.__pose.x*40+40, self.__pose.y*40+40
            self.__path = self.__planningManager.calculate(self.__mapData, [[0, w], [0, h]], (x, y))
            self.__path = np.array(self.__path) / 40 - 1

    def control(self) -> None:
        if self.__path.size > 0:
            pos, self.__path = self.__path[0], self.__path[1:]
            self.__controlManager.follow(pos, (self.__pose.x, self.__pose.y), 0.0)
            self.__velocity.linear.x = self.__controlManager.getLinearVel()
            self.__velocity.angular.z = self.__controlManager.getAngularVel()
            self.__vel_pub.publish(self.__velocity)

    # Stop function
    def _stop(self) -> None:
        print("Stopping the path planning")
 
if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("Path_Planning")

    # Initialize the rate
    rate = rospy.Rate(rospy.get_param("rateObst", default = 15))

    # Get parameters
    samples = rospy.get_param("planning/samples/value", default = 500)
    maxDist = rospy.get_param("planning/maxDistance/value", default = 5.0)
    density = rospy.get_param("planning/pointsDensity/value", default = 100)

    # Create the instance of the class
    pathHandler = pathPlanning(samples, maxDist, density)

    # Shutdown hook
    rospy.on_shutdown(pathHandler._stop)

    # Run the node
    print("The Path Planner is Running")
    while not rospy.is_shutdown():
        try:
            pathHandler.control()
            rate.sleep()
        except rospy.ROSInterruptException as ie:
            rospy.loginfo(ie) # Catch an Interruption
