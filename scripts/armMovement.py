#!/usr/bin/python3
import rospy
import numpy as np
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import sys
sys.path.append("/home/tavo/Ciberfisicos_ws/src/jet_auto2/scripts")
from hiwonder_servo_msgs.msg import CommandDuration

from Classes.robotArm import robotArm

# Class to move the arm
class armMovement:
    def __init__(self, links:list, vel:float) -> None:
        # Instance of the robot arm class
        self.__jointsManager = robotArm([link for link in links])
        # Real time Joints state
        self.__jointsPosition = [0.0, -np.pi/4, 5*np.pi/8, 3*np.pi/8, 0.0]
        # Desired velocity for all servos (rad/s)
        self.__vel = vel

        # Flag to grab the object
        self.__grab = False
        # Gripper close position
        self.__close = np.pi/4
        # Gripper open position
        self.__open = 2*np.pi/3
        # Joints coords for dropping an object
        self.__dropCoords = Point(0.0, -0.2, 0.1)

        # Initialize the subscribers and publishers
        rospy.Subscriber("/object/coords", Point, self.__coordsCallback)
        rospy.Subscriber("/object/grab", Bool, self.__grabCallback)
        rospy.Subscriber("/object/drop", Bool, self.__dropCallback)
        rospy.Subscriber('/joint_states', JointState, self.__statesCallback)
        self.__joint1_pub = rospy.Publisher('/joint1_controller/command_duration', CommandDuration, queue_size = 10)
        self.__joint2_pub = rospy.Publisher('/joint2_controller/command_duration', CommandDuration, queue_size = 10)
        self.__joint3_pub = rospy.Publisher('/joint3_controller/command_duration', CommandDuration, queue_size = 10)
        self.__joint4_pub = rospy.Publisher('/joint4_controller/command_duration', CommandDuration, queue_size = 10)
        self.__gripper_pub = rospy.Publisher('/r_joint_controller/command_duration', CommandDuration, queue_size = 10)
        self.__return_pub = rospy.Publisher('/return', Bool, queue_size = 1)

    # Callback function for the coordinates of the object
    def __coordsCallback(self, msg:Point) -> None:
        x, y, z = msg.x, msg.y, msg.z # Get the coordinates of the object (m).
        self.__jointsManager._inverseKinematics(x, y, z) # Calculate the inverse kinematics
        joints = self.__jointsManager.getJoints() # Get the joint angles
        self.jointsPublish(joints) if joints else None # Publish the joints commands

    # Callback function for the grabbing variable state
    def __grabCallback(self, msg:Bool) -> None:
        self.__grab = msg.data # Get the current state
    
    # Callback function for the dropping variable state 
    def __dropCallback(self, msg:Bool) -> None:
        if msg.data is True:
            self._dropObject()

    # Callback function for the states of the joints
    def __statesCallback(self, msg:JointState) -> None:
        self.__jointsPosition = msg.position # Get the position of all joints

    # Publish the joints commands
    def jointsPublish(self, joints:list) -> None:
        # Get the time (ms) for the servos to move
        t1, t2, t3, t4 = self.__adjustTime(np.abs(np.array(self.__jointsPosition[:-1]) - np.array(joints)) / self.__vel * 1000.0)
        self.__joint1_pub.publish(joints[0], t1) # Publish the joint 1 data
        self.__joint2_pub.publish(joints[1], t2) # Publish the joint 2 data
        self.__joint3_pub.publish(joints[2], t3) # Publish the joint 3 data
        self.__joint4_pub.publish(joints[3], t4) # Publish the joint 4 data
        if self.__grab is not None:
            gripperCommand = self.__close if self.__grab is True else self.__open
            self.gripperPublish(gripperCommand, (t1, t2, t3, t4)) # Publish the gripper data
            self._afterGrab() if self.__grab == True else None # Move the arm after grabbing an object
        else:
            self.__return_pub.publish(True) # Publish the return flag

    # Publish the gripper command
    def gripperPublish(self, command:float, jointsTime:list) -> None:
        rospy.sleep(max(jointsTime) / 1000.0) # Wait for the arm joints to move
        t5 = abs(self.__jointsPosition[-1] - command) / self.__vel * 1000.0 # Get time for moving the gripper
        self.__gripper_pub.publish(command, t5)
        rospy.sleep(t5/1000.0) # Wait for the gripper to close

    # Adjust the time for each joint according to grabbing state
    def __adjustTime(self, time:np.ndarray) -> np.ndarray:
        time[0] *= 3 if self.__grab is None else 1 # Increase time for moving the first joint
        time[1] *= 3 if self.__grab is True else 1 # Increase time for moving the second joint
        time[3] *= 2 if self.__grab is True else 1 # Increase time for moving the fourth joint
        return time

    # Function to move the arm after grabbing an object
    def _afterGrab(self) -> None:
        print("Grabbing the object")
        self.__grab = None
        joints = self.__jointsManager._grabPosition() # Get the joints angles
        self.jointsPublish(joints)

    # Function to move the arm to a drop position
    def _dropObject(self) -> None:
        print("Dropping the object")
        self.__grab = False # Open gripper
        self.__coordsCallback(self.__dropCoords) # Move arm to drop position
        self._start() # Return arm to start position

    # Function to set a start position
    def _start(self) -> None:
        print("The Arm Movement node is Running")
        joints = self.__jointsManager._startArm() # Get the joints angles
        self.jointsPublish(joints)

    # Reset the arm position when the node is shutdown
    def _stop(self) -> None:
        print("Stopping the Arm Movement node")
        # self._dropObject()
        self.__grab = False
        joints = self.__jointsManager._resetArm() # Get the joints angles
        self.jointsPublish(joints)

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('Arm_Movement')

    # Initialize the rate
    rate = rospy.Rate(rospy.get_param("rateClass", default = 15))

    # Get the parameters
    l1 = rospy.get_param("links/link1/lenght", default = 0.03)
    l2 = rospy.get_param("links/link2/lenght", default = 0.13)
    l3 = rospy.get_param("links/link3/lenght", default = 0.13)
    l4 = rospy.get_param("links/link4/lenght", default = 0.05)
    vel = rospy.get_param("vel", default = 0.2)

    # Create the instance of the class
    roboticArm = armMovement([l1, l2, l3, l4], vel)
    rospy.sleep(1.0)
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
