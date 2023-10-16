#!/usr/bin/python3
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from Classes.camera import Camera

if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("Camera")

    hz = 10 # Frequency (Hz)
    rate = rospy.Rate(hz)

    cam = Camera(0) # Camera class object

    # Publishers and subscribers
    camera_pub = rospy.Publisher("/video_source/raw", Image, queue_size = 10) # Publish the camera image

    print("The Camera is Running")
    rospy.on_shutdown(cam._releaseCamera)

    # Run the node
    while not rospy.is_shutdown():
        try:
            frame = cam.getImage()
            if frame:
                image = cv_bridge.CvBridge().cv2_to_imgmsg(frame, encoding = 'rgb8') # Convert from open cv to imgmsg
                camera_pub.publish(image) # Publish the current frame
        except rospy.ROSInterruptException:
            pass

        rate.sleep()
