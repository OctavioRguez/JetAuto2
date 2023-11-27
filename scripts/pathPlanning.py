#!/home/sr_tavo/venv/bin/python3
import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Odometry
import matplotlib.pyplot as plt
import numpy as np
import sys
sys.path.append("/home/tavo/Ciberfisicos_ws/src/jet_auto2/scripts")
from Classes.prmAstar import PRMAstar

# Avoid obstacles based on LiDAR data
class pathPlanning:
    def __init__(self, points:int, dist:float, density:int) -> None:
        # Create the obstacle avoidance instance
        self.__planningManager = PRMAstar(points, dist, density)
        # Map with obstacles from the SLAM
        self.__mapData = None
        # Position estimated with odometry
        self.__pose = None
        
        # Initialize the subscribers and publishers
        rospy.Subscriber("/map", OccupancyGrid, self.__mapCallback)
        rospy.Subscriber("/odom", Odometry, self.__odomCallback)
        rospy.Subscriber("/initiate", Bool, self.__initCallback)

    # Callback function for getting the lidar data
    def __mapCallback(self, msg:OccupancyGrid) -> None:
        self.__mapData = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        # Plot map
        plt.figure()
        plt.imshow(self.__mapData, cmap='Greys', origin='lower')
        plt.savefig("/home/sr_tavo/map.png")

    def __odomCallback(self, msg:Odometry) -> None:
        self.__pose = msg.pose
        rospy.loginfo(self.__pose)

    def __initCallback(self, msg:Bool) -> None:
        if msg.data is True:
            self.start()

    def start(self) -> None:
        if self.__mapData is not None and self.__pose is not None:
            w, h = self.__mapData.shape
            self.__planningManager.calculate(self.__mapData, [[0, w], [0, h]], self.__pose)

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
            rate.sleep()
        except rospy.ROSInterruptException as ie:
            rospy.loginfo(ie) # Catch an Interruption
