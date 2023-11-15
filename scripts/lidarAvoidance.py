#!/usr/bin/python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
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

        # Initialize the subscribers and publishers
        rospy.Subscriber("/scan", LaserScan, self.__scanCallback)
        rospy.Subscriber("/object/coords", Point, self.__coordsCallback)
        self.__vel_pub = rospy.Publisher("/jetauto_controller/cmd_vel", Twist, queue_size = 10)

    def __scanCallback(self, data:LaserScan) -> None:
        if self.__navegate:
            # Check for obstacles
            self.__obstacleManager._avoidObstacles(data.ranges)
            # Get linear and angular velocities
            self.__velocity.linear.x = self.__obstacleManager.getLinear()
            self.__velocity.angular.z = self.__obstacleManager.getAngular()
            self.__vel_pub.publish(self.__velocity) # Publish the velocity

    def __coordsCallback(self, data:Point) -> None:
        vel = 0.0 if data.x>0.22 else 0.04
        self.__obstacleManager.changeLinearVelocity(vel, vel)
        self.__obstacleManager.changeAngularVelocity(0.0, 0.0)

    def _stop(self) -> None:
        # Stop the robot
        print("Stopping the obstacle avoidance")
        self.__velocity.linear.x, self.__velocity.angular.z = 0.0, 0.0
        self.__vel_pub.publish(self.__velocity) # Publish the velocity
 
if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("Lidar_Obstacle_Avoidance")

    # Initialize the rate
    rate = rospy.Rate(rospy.get_param("rateObst", default = 15))

    # Get parameters
    safeDist = rospy.get_param("safe_distance", default = 0.5)

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
