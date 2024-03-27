#!/usr/bin/python3
import rospy
import numpy as np
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import sys
sys.path.append("/home/tavo/Ciberfisicos_ws/src/jet_auto2/scripts")
from Classes.prmAstar import PRMAstar
from Classes.controlTraj import OmnidirectionalDriveRobot

# Avoid obstacles based on LiDAR data
class pathPlanning:
    def __init__(self, points:int, dist:float, density:int) -> None:
        # Create the obstacle avoidance instance
        self.__planningManager = PRMAstar(points, dist, density)
        # Map with obstacles from the SLAM
        self.__mapData = None
        # Position and angle estimated with odometry
        self.__pose = None
        self.__angle = None

        # Create the control manager
        self.__controlManager = OmnidirectionalDriveRobot(0.3, 0.2, 0.07, 1.2)
        self.__velocity = Twist()
        # Control constants
        self.__kpr = 0.6
        self.__wmax = 0.3
        self.__angThreshold = np.pi/20

        # Path variables
        self.__path = np.array([])
        self.__next = False # Flag to move to the next point
        self.__return = False # Flag to activate the planning
        
        # Initialize the subscribers and publishers
        rospy.Subscriber("/map", OccupancyGrid, self.__mapCallback, queue_size = 1)
        rospy.Subscriber("/odom", Odometry, self.__odomCallback)
        rospy.Subscriber("/return", Bool, self.__returnCallback)
        self.__vel_pub = rospy.Publisher("/jetauto_controller/cmd_vel", Twist, queue_size = 10)
        self.__drop_pub = rospy.Publisher("/object/drop", Bool, queue_size = 1)

    # Callback function for getting the lidar data
    def __mapCallback(self, msg:OccupancyGrid) -> None:
        self.__mapData = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    # Callback function for getting the current odometry position
    def __odomCallback(self, msg:Odometry) -> None:
        self.__pose = msg.pose.pose.position
        ang = msg.pose.pose.orientation
        self.__angle = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])[2]

    # Callback function for getting the flag for starting the path planning
    def __returnCallback(self, msg:Bool) -> None:
        if msg.data:
            # Calculate the PRM
            w, h = self.__mapData.shape
            self.__planningManager.calculate(self.__mapData, [(0, h), (0, w)])

            # Find the path
            x, y = (self.__pose.x+1)*40, (self.__pose.y+1)*40
            self.__path = self.__planningManager.findPath((x, y))
            self.__path = [(x/40-1, y/40-1) for x, y in self.__path] # Convert the path to the real coordinates

    # Control function
    def control(self) -> None:
        if len(self.__path) > 0:
            self.__return = True # Activate the planning
            # Angular control
            thetae = self.__angle
            angVel = self.__wmax*np.tanh(-self.__kpr * thetae/self.__wmax)
            print(self.__angle)

            # Start omnidirectional control
            if abs(thetae) < self.__angThreshold:
                currGoal = self.__path[0]
                # Check if the robot has reached the current goal
                self.__next = self.__controlManager.follow(currGoal, (self.__pose.x, self.__pose.y))
                self.__path = self.__path[1:] if self.__next else self.__path

                # Publish the velocity
                self.velocityPublish(self.__controlManager.getXVel(), self.__controlManager.getYVel(), angVel)
            else:
                self.velocityPublish(0.0, 0.0, angVel)
        elif self.__return: # Finish the path
            rospy.sleep(1.0)
            self.velocityPublish(0.0, 0.0, 0.0) # Stop the robot
            self.__return = False # Deactivate the planning
            self.__drop_pub.publish(True) # Drop the object
    
    # Public function for publishing the velocity
    def velocityPublish(self, x:float, y:float, z:float) -> None:
        self.__velocity.linear.x, self.__velocity.linear.y = x, y
        self.__velocity.angular.z = z
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
