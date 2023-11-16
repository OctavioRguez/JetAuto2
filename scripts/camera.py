#!/home/sr_tavo/venv/bin/python3
import rospy
import cv2 as cv
from sensor_msgs.msg import CompressedImage

class Camera:
    def __init__(self) -> None:
        self.__camera_pub = rospy.Publisher("/usb_cam/image_raw/compressed", CompressedImage, queue_size = 10)
        self.__cap = cv.VideoCapture(0)

        self.__compressedImg = CompressedImage()
        self.__compressedImg.format = "jpeg"

    def _start(self) -> None:
        if self.__cap.isOpened():
            ret, frame = self.__cap.read() # Get actual frame
            if ret:
                # Create a CompressedImage message
                self.__compressedImg.data = cv.imencode('.jpg', frame)[1].tobytes()
                # Publish the compressed image
                self.__camera_pub.publish(self.__compressedImg)
            else:
                print("Unable to obtain current frame")
        else:
            print("Error: Could not open camera")

    # Stop Condition
    def _stop(self) -> None:
        self.__cap.release()
        print("Stopping classificator node")

if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("Camera")

    # Initialize the rate
    rate = rospy.Rate(30)

    # Create the instance of the class
    handler = Camera()

    # Shutdown hook
    rospy.on_shutdown(handler._stop)

    # Run the node
    print("The Camera is Running")
    while not rospy.is_shutdown():
        try:
            handler._start()
        except rospy.ROSInterruptException as ie:
            rospy.loginfo(ie) # Catch an Interruption

        rate.sleep()
