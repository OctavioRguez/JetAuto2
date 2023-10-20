#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Point
from hiwonder_servo_msgs.msg import CommandDuration
from Classes.inverseKinematics import inverseKinematics

# Publish the joints commands
def jointsPublish(joints:list, duration:float) -> None:
    joint1_pub.publish(joints[0], duration) # Publish the joint 1 data
    joint2_pub.publish(joints[1], duration) # Publish the joint 2 data
    joint3_pub.publish(joints[2], duration) # Publish the joint 3 data
    joint4_pub.publish(joints[3], duration) # Publish the joint 4 data

# Callback function for the coordinates of the object
def coordsCallback(msg:Point) -> None:
    x, y, z = msg.x, msg.y, msg.z # Get the coordinates of the object
    if x < 0.2:
        ik._inverseKinematics(x, y, z) # Calculate the inverse kinematics
        joints = ik.getJoints() # Get the joint angles
        jointsPublish(joints, 2500.0)

# Stop the arm when the node is shutdown
def stop() -> None:
    print("Stopping") # Stop message
    joints = ik._reset() # Reset the robotic arm
    jointsPublish(joints, 2500.0)

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('Arm_Movement')

    hz = 10 # Frequency (Hz)
    rate = rospy.Rate(hz)

    ik = inverseKinematics(0.03, 0.13, 0.13, 0.05) # Inverse Kinematics class object

    # Initialize the subscribers and publishers
    rospy.Subscriber("/object/coords", Point, coordsCallback)
    joint1_pub = rospy.Publisher('/joint1_controller/command_duration', CommandDuration, queue_size = 1)
    joint2_pub = rospy.Publisher('/joint2_controller/command_duration', CommandDuration, queue_size = 1)
    joint3_pub = rospy.Publisher('/joint3_controller/command_duration', CommandDuration, queue_size = 1)
    joint4_pub = rospy.Publisher('/joint4_controller/command_duration', CommandDuration, queue_size = 1)

    rospy.sleep(2.0)
    joints = ik._start() # Start the robotic arm
    jointsPublish(joints, 2000.0)
    print("The Arm Movement node is Running")
    rospy.on_shutdown(stop)

    # Run the node
    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except rospy.ROSInterruptException as ie:
            rospy.loginfo(ie) # Catch an Interruption
