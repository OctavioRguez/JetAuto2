#!/usr/bin/python3
import rospy
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from hiwonder_servo_msgs.msg import CommandDuration
from Classes.robotArm import robotArm

# Class to move the arm
class armMovement:
    def __init__(self, links:list, vel:float) -> None:
        # Instance of the robot arm class
        self.__jointsManager = robotArm(link for link in links)
        # Real time Joints state
        self.__jointsPosition = [0.0, 0.0, 0.0, 0.0]
        # Desired velocity for all servos (rad/s)
        self.__vel = vel

        # Initialize the subscribers and publishers
        rospy.Subscriber("/object/coords", Point, self.__coordsCallback)
        rospy.Subscriber('/joint/states', JointState, self.__statesCallback)
        self.__joint1_pub = rospy.Publisher('/joint1_controller/command_duration', CommandDuration, queue_size = 1)
        self.__joint2_pub = rospy.Publisher('/joint2_controller/command_duration', CommandDuration, queue_size = 1)
        self.__joint3_pub = rospy.Publisher('/joint3_controller/command_duration', CommandDuration, queue_size = 1)
        self.__joint4_pub = rospy.Publisher('/joint4_controller/command_duration', CommandDuration, queue_size = 1)

    # Callback function for the coordinates of the object
    def __coordsCallback(self, msg:Point) -> None:
        x, y, z = msg.x, msg.y, msg.z # Get the coordinates of the object (m)
        if x < 0.25:
            self.__jointsManager._inverseKinematics(x, y, z) # Calculate the inverse kinematics
            joints = self.__jointsManager.getJoints() # Get the joint angles
            self.jointsPublish(joints)

    # Callback function for the states of the joints
    def __statesCallback(self, msg:JointState) -> None:
        self.__jointsPosition = msg.position # Get the position of all joints

    # Publish the joints commands
    def jointsPublish(self, joints:list) -> None:
        # Get the time (ms) for the servos to move
        t1, t2, t3, t4 = np.abs(np.array(self.__jointsPosition) - np.array(joints)) / self.__vel * 1000
        self.__joint1_pub.publish(joints[0], t1) # Publish the joint 1 data
        self.__joint2_pub.publish(joints[1], t2) # Publish the joint 2 data
        self.__joint3_pub.publish(joints[2], t3) # Publish the joint 3 data
        self.__joint4_pub.publish(joints[3], t4) # Publish the joint 4 data

    # Function to set a start position
    def _start(self) -> None:
        print("The Arm Movement node is Running")
        joints = self.__jointsManager._startArm()
        self.jointsPublish(joints)

    # Reset the arm position when the node is shutdown
    def _stop(self) -> None:
        print("Stopping the Arm Movement node")
        joints = self.__jointsManager._resetArm()
        self.jointsPublish(joints)

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('Arm_Movement')

    # Initialize the rate
    rate = rospy.Rate(rospy.get_param("rate", default = 10))

    # Get the parameters
    l1 = rospy.get_param("links/link1/lenght", default = 0.03)
    l2 = rospy.get_param("links/link2/lenght", default = 0.13)
    l3 = rospy.get_param("links/link3/lenght", default = 0.13)
    l4 = rospy.get_param("links/link4/lenght", default = 0.05)
    vel = rospy.get_param("vel", default = 1.0)

    # Create the instance of the class
    roboticArm = armMovement([l1, l2, l3, l4], vel)
    # Wait for the servos controller to start
    rospy.sleep(2.0)
    # Move the arm to a start position
    roboticArm._start()

    # Shutdown hook
    rospy.on_shutdown(roboticArm._stop)

    # Run the node
    while not rospy.is_shutdown():
        try:
            rate.sleep() # Sleep for the remainder of the loop
        except rospy.ROSInterruptException as e:
            rospy.loginfo(e) # Catch an Interruption
