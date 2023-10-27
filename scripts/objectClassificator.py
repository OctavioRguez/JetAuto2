#!/usr/bin/python3
import rospy
import cv_bridge
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from Classes.modelPredict import modelPredict

class objectClassificator:
    def __init__(self, model:str, classes:list, conf_threshold:float, cuda:bool) -> None:
        # Instance of the class modelPredict
        self.__model = modelPredict(model, classes, conf_threshold, cuda)

        # Initialize the variables
        self.__img = None
        self.__object = None # Desired Class name of the object
        self.__objWidth, self.__objHeight = 0.05, 0.1 # Object dimensions (m)

        # Initialize the subscribers and publishers
        rospy.Subscriber("/video_source/raw", Image, self.__imageCallback) # Get the image from the camera
        rospy.Subscriber("/object/class", String, self.__classCallback) # Get the class of the object
        self.__coord_pub = rospy.Publisher("/object/coords", Point, queue_size = 10) # Publish the coordinates of the object (m)

    # Callback funtion for the image
    def __imageCallback(self, msg:Image) -> None:
        try:
            self.__img = cv_bridge.CvBridge().imgmsg_to_cv(msg, desired_encoding = 'passthrough') # Conversion from imgmsg to cv
        except cv_bridge.CvBridgeError as e:
            rospy.loginfo(e) # Catch an error

    # Callback function for the class name
    def __classCallback(self, msg:String) -> None:
        self.__object = msg.data

    # Start the model classification
    def _startModel(self) -> None:
        if (self.__img is not None) and (self.__object is not None):
            self.__model._startDetection(self.__img, self.__object, self.__objWidth) # Detect on current frame
            self.__coord_pub.publish(0.2, self.__model.getY(), -0.17 + self.__objHeight/2)

    # Stop Condition
    def _stop(self) -> None:
        print("Stopping classificator node")

if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("Classificator")

    # Initialize the rate
    rate = rospy.Rate(rospy.get_param("rate", default = 10))

    # Get the parameters
    model = rospy.get_param("model/path", default = "../Model/bestV5-25e.onnx")
    class_list = rospy.get_param("classes/list", default = ["Fanta", "Pepsi", "Seven"])
    conf = rospy.get_param("confidence/value", default = 0.5)
    cuda = rospy.get_param("isCuda/value", default = False)

    # Create the instance of the class
    classificator = objectClassificator(model, class_list, conf, cuda)

    # Shutdown hook
    rospy.on_shutdown(classificator._stop)

    # Run the node
    print("The Classificator is Running")
    while not rospy.is_shutdown():
        try:
            classificator._startModel()
        except rospy.ROSInterruptException as ie:
            rospy.loginfo(ie) # Catch an Interruption
        
        rate.sleep()
