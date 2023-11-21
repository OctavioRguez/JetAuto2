#!/home/sr_tavo/venv/bin/python3
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
# import sys
# sys.path.append("/home/tavo/Ciberfisicos_ws/src/JetAuto2/scripts")
from Classes.modelPredict import modelPredict

class objectClassificator:
    def __init__(self, model:str, classes:list, conf_threshold:float, cuda:bool) -> None:
        # Instance of the class modelPredict
        self.__model = modelPredict(model, classes, conf_threshold, cuda)

        # Initialize the variables
        self.__img = None
        self.__object = None # Desired Class name of the object
        self.__objWidth, self.__objHeight = 0.06, 0.12 # Object dimensions (m)

        self.__armCoords = {"x":0.22, "y":0.0, "z":self.__objHeight/2 - 0.03} # Current end effector coordinates of the arm (m)
        self.__lastY = 0.0 # Last y coordinate of the object (m)
        self.__tolerance = 10 # Tolerance for capturing the object movement (iterations)
        self.__errorTolerance = 0.01 # Tolerance for the error (m)
        self.__grab = False # Flag for check if the object is grabbed

        self.__kp = 0.7 # Proportional constant for the arm movement

        # Compressed image message
        self.__compressedImg = CompressedImage()
        self.__compressedImg.format = "jpeg"

        # Initialize the subscribers and publishers
        rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.__imageCallback) # Get the image from the camera
        rospy.Subscriber("/object/class", String, self.__classCallback) # Get the class of the object
        rospy.Subscriber("/object/drop", Bool, self.__dropCallback) # Get the drop flag
        self.__coord_pub = rospy.Publisher("/object/coords", Point, queue_size = 1) # Publish the coordinates of the object (m)
        self.__grab_pub = rospy.Publisher("/object/grab", Bool, queue_size = 1) # Publish if the object is ready to be grabbed
        self.__detection_pub = rospy.Publisher("/usb_cam/model_prediction/compressed", CompressedImage, queue_size = 10) # Publish the image with the prediction

    # Callback funtion for the image
    def __imageCallback(self, msg:CompressedImage) -> None:
        self.__img = msg.data

    # Callback function for the class name
    def __classCallback(self, msg:String) -> None:
        self.__object = msg.data
 
    def __dropCallback(self, msg:Bool) -> None:
        self.__grab = False if msg.data else self.__grab

    # Start the model classification
    def _startModel(self) -> None:
        if self.__img is not None and not self.__grab:
            # Detect on current frame
            decodedImg = self.__model._startDetection(self.__img, self.__object, self.__objWidth)
            y, depth = self.__model.getHorizontal(), self.__model.getDepth() # Get the coordinates of the object

            # Move the arm
            self.__moveArm(y, depth) if y is not None else None

            # Publish the compressed image
            self.__compressedImg.data = decodedImg
            self.__detection_pub.publish(self.__compressedImg)

    def __moveArm(self, y:float, depth:float) -> None:
        # Tolerance is increased if the object has not moved
        self.__tolerance += 1 if (-1e-3 < (self.__lastY - y) < 1e-3 or not self.__tolerance) else -1
        self.__lastY = y

        # Check if the object has not moved for "n" iterations
        if (self.__tolerance > 3):
            # Check if the y coordinate is close to zero (middle of the image)
            if not(-self.__errorTolerance < y < self.__errorTolerance):
                self.__armCoords["y"] += y*self.__kp
                self.__tolerance = 0
                self.__coord_pub.publish(self.__armCoords["x"], self.__armCoords["y"], self.__armCoords["z"])
            # Check if the x coordinate is close enough to grab the object
            else:
                x = (depth**2 - self.__armCoords["z"]**2)**0.5
                rospy.loginfo(x)
                if False: #x < 0.17:
                    self.__grab, self.__object = True, None
                    self.__tolerance = 0
                    self.__grab_pub.publish(True)
                    # rospy.sleep(1.0)
                    self.__coord_pub.publish(self.__armCoords["x"]+(x-0.08), self.__armCoords["y"], self.__armCoords["z"]+0.03)

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
