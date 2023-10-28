#!/usr/bin/python3
import rospy
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from Classes.robotArm import robotArm

# Class to move the arm
class armTrajectory:
    def __init__(self, links:list, rate:int) -> None:
        # Instance of the robot arm class
        self.__jointsManager = robotArm([link for link in links])
        # Real time Joints state
        self.__jointsPosition = [0.0, -np.pi/4, 5*np.pi/8, 3*np.pi/8]
        # Trajectory for the arm
        self.__trajectory = np.array([])
        # Rate for calculating the trajectory
        self.__rate = rate

        # Initialize the subscribers and publishers
        rospy.Subscriber("/object/coords", Point, self.__coordsCallback)
        rospy.Subscriber('/joint_states', JointState, self.__statesCallback)
        self.__joint1_pub = rospy.Publisher('/joint1_controller', Float32, queue_size = 10)
        self.__joint2_pub = rospy.Publisher('/joint2_controller', Float32, queue_size = 10)
        self.__joint3_pub = rospy.Publisher('/joint3_controller', Float32, queue_size = 10)
        self.__joint4_pub = rospy.Publisher('/joint4_controller', Float32, queue_size = 10)

    # Callback function for the coordinates of the object
    def __coordsCallback(self, msg:Point) -> None:
        x, y, z = msg.x, msg.y, msg.z # Get the coordinates of the object (m)
        if x < 0.25:
            self.__jointsManager._inverseKinematics(x, y, z) # Calculate the inverse kinematics
            joints = self.__jointsManager.getJoints() # Get the joint angles

            # Calculate the trajectory and flip it to get the goal position at the beginning
            self.__trajectory = np.flip(self.__jointsManager._trajectory(self.__jointsPosition, joints, self.__rate))

    # Callback function for the states of the joints
    def __statesCallback(self, msg:JointState) -> None:
        self.__jointsPosition = msg.position[:-1] # Get the position of all joints

    # Get the next position of the trajectory
    def getNextPosition(self) -> list:
        if self.__trajectory.size > 0: # Check if the trajectory is not empty
            return self.__trajectory.pop()
        return None

    # Publish the joints commands
    def jointsPublish(self, joints:list) -> None:
        self.__joint1_pub.publish(joints[0]) # Publish the joint 1 data
        self.__joint2_pub.publish(joints[1]) # Publish the joint 2 data
        self.__joint3_pub.publish(joints[2]) # Publish the joint 3 data
        self.__joint4_pub.publish(joints[3]) # Publish the joint 4 data

    # Function to set a start position
    def _start(self) -> None:
        print("The Arm Movement node is Running")
        joints = self.__jointsManager._startArm()
        self.__trajectory = np.flip(self.__jointsManager._trajectory(self.__jointsPosition, joints, self.__rate))

    # Reset the arm position when the node is shutdown
    def _stop(self) -> None:
        print("Stopping the Arm Movement node")
        joints = self.__jointsManager._resetArm()
        self.__trajectory = np.flip(self.__jointsManager._trajectory(self.__jointsPosition, joints, self.__rate))

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('Arm_Movement')

    # Initialize the rate
    rate = rospy.Rate(10)

    # Get the parameters
    l1 = rospy.get_param("links/link1/lenght", default = 0.03)
    l2 = rospy.get_param("links/link2/lenght", default = 0.13)
    l3 = rospy.get_param("links/link3/lenght", default = 0.13)
    l4 = rospy.get_param("links/link4/lenght", default = 0.05)
    ratio = rospy.get_param("rate", default = 30)

    # Create the instance of the class
    roboticArm = armTrajectory([l1, l2, l3, l4], ratio)
    # Wait for the servos controller to start
    rospy.sleep(2.0)
    # Move the arm to a start position
    roboticArm._start()

    # Shutdown hook
    rospy.on_shutdown(roboticArm._stop)

    # Run the node
    while not rospy.is_shutdown():
        try:
            joints = roboticArm.getNextPosition()
            if joints is not None:
                roboticArm.jointsPublish(joints)
        except rospy.ROSInterruptException as e:
            rospy.loginfo(e) # Catch an Interruption
    rate.sleep()
