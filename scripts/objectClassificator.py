#!/usr/bin/python3
import rospy
import cv_bridge
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from Classes.modelPredict import modelPredict

img, object = None, None

def imageCallback(msg) -> None:
    try:
        global img
        img = cv_bridge.CvBridge().imgmsg_to_cv(msg, desired_encoding = 'passthrough') # Conversion from imgmsg to cv
    except cv_bridge.CvBridgeError as e:
        rospy.loginfo(e) # Catch an error

def classCallback(msg) -> None:
    global object
    object = msg.data

# Stop Condition
def stop() -> None:
    print("Stopping") # Stop message

if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("Classificator")

    hz = 10 # Frequency (Hz)
    rate = rospy.Rate(hz)

    # Classificator class object
    model = modelPredict("../Model/bestV4-40e.onnx", ["Fanta", "Pepsi", "Seven"], [(255, 0, 0), (0, 255, 0), (0, 0, 255)])

    # Publishers and subscribers
    rospy.Subscriber("/video_source/raw", Image, imageCallback) # Get the image from the camera
    rospy.Subscriber("/object/class", String, classCallback) # Get the class of the object
    coord_pub = rospy.Publisher("/object/coords", Point, queue_size = 1)

    print("The Classificator is Running")
    rospy.on_shutdown(stop)

    # Run the node
    while not rospy.is_shutdown():
        try:
            if (img is not None) and (object is not None):
                model._startDetection(object, img, 0.1)
                coord_pub.publish(0.2, model.getY(), -0.1)
        except rospy.ROSInterruptException as ie:
            rospy.loginfo(ie) # Catch an Interruption
            pass
        
        rate.sleep()
    
