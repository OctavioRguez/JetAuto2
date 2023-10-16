#!/usr/bin/python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from Classes.inverseKinematics import inverseKinematics

# Callback function for the coordinates of the object
def coordsCallback(msg:Point) -> None:
    ik._inverseKinematics(msg.x, msg.y, msg.z) # Calculate the inverse kinematics

# Stop the arm when the node is shutdown
def stop() -> None:
    joint1_pub.publish(0.0)
    joint2_pub.publish(0.0)
    joint3_pub.publish(0.0)
    joint4_pub.publish(0.0)
    print("Stopping")

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('Arm_Movement')

    hz = 10 # Frequency (Hz)
    rate = rospy.Rate(hz)

    ik = inverseKinematics(0.03, 0.12, 0.13, 0.05) # Inverse Kinematics class object

    # Initialize the subscribers and publishers
    rospy.Subscriber("/object/coords", Point, coordsCallback)
    joint1_pub = rospy.Publisher('/joint1_controller/command', Float64, queue_size = 10)
    joint2_pub = rospy.Publisher('/joint2_controller/command', Float64, queue_size = 10)
    joint3_pub = rospy.Publisher('/joint3_controller/command', Float64, queue_size = 10)
    joint4_pub = rospy.Publisher('/joint4_controller/command', Float64, queue_size = 10)

    print("The Arm Movement node is Running")
    rospy.on_shutdown(stop)

    # Run the node
    while not rospy.is_shutdown():
        try:
            joints = ik.getJoints() # Get the joint angles

            joint1_pub.publish(Float64(joints[0])) # Publish the joint 1 data
            joint2_pub.publish(Float64(joints[1])) # Publish the joint 2 data
            joint3_pub.publish(Float64(joints[2])) # Publish the joint 3 data
            joint4_pub.publish(Float64(joints[3])) # Publish the joint 4 data
        except rospy.ROSInterruptException:
            pass
        
        rate.sleep()
