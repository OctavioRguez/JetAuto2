#!/home/sr_tavo/venv/bin/python3
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
from Classes.modelPredict import modelPredict

class objectClassificator:
    def __init__(self, model:str, classes:list, conf_threshold:float, cuda:bool) -> None:
        # Instance of the class modelPredict
        self.__model = modelPredict(model, classes, conf_threshold, cuda)

        # Initialize the variables
        self.__img = None
        self.__object = None # Desired Class name of the object
        self.__baseHeight = 0.1 # Base height for the object (m)
        self.__objWidth, self.__objHeight = 0.065, 0.12 # Object dimensions (m)

        self.__robotY = 0.0 # Current y coordinate of the arm
        self.__lastY = 0.0 # Last y coordinate of the object
        self.__tolerance = 0 # Tolerance for getting the object velocity
        self.__errorTolerance = 0.01 # Tolerance for the error (m)

        # Initialize the subscribers and publishers
        rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.__imageCallback) # Get the image from the camera
        rospy.Subscriber("/object/class", String, self.__classCallback) # Get the class of the object
        self.__coord_pub = rospy.Publisher("/object/coords", Point, queue_size = 1) # Publish the coordinates of the object (m)
        self.__grab_pub = rospy.Publisher("/object/grab", Bool, queue_size = 1) # Publish if the object is ready to be grabbed

    # Callback funtion for the image
    def __imageCallback(self, msg:CompressedImage) -> None:
        self.__img = msg.data

    # Callback function for the class name
    def __classCallback(self, msg:String) -> None:
        self.__object = msg.data

    # Start the model classification
    def _startModel(self) -> None:
        if self.__img is not None:
            # Detect on current frame
            self.__model._startDetection(self.__img, self.__object, self.__objWidth)
            x, y = self.__model.getX(), self.__model.getY() # Get the coordinates of the object

            if (y is not None):
                # Update the tolerance
                self.__tolerance += 1 if -self.__errorTolerance < (self.__lastY - y) < self.__errorTolerance else -self.__tolerance
                self.__lastY = y

                # Check if the object has not moved for a while
                if (self.__tolerance > 10):
                    # Check if the y coordinate is close to zero (middle of the image)
                    if not(-self.__errorTolerance < y < self.__errorTolerance):
                        self.__robotY += y
                        self.__coord_pub.publish(0.2, self.__robotY, -0.17+self.__baseHeight+self.__objHeight/2)
                    elif (x is not None):
                        self.__coord_pub.publish(0.2+(x-0.05), self.__robotY, -0.17+self.__base+self.__objHeight/2)
                        self.__grab_pub.publish(True)
                    self.__tolerance = 0

    # Stop Condition
    def _stop(self) -> None:
        print("Stopping classificator node")

if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("Classificator")

    # Initialize the rate
    rate = rospy.Rate(rospy.get_param("rateClass", default = 15))

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
