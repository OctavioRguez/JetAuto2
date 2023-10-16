# JetAuto2

JetAuto2 is a ROS (Robot Operating System) project for controlling the Hiwonder JetAuto Pro Vehicle. It is designed to enable the vehicle to move autonomously, using SLAM for mapping and obstacle avoidance. Additionally, the vehicle is equipped with a built-in robotic arm that can be used to grab objects and this arm is controlled by an AI model that detects and classifies objects. Finally, planning and navegation algorithms are also used in order to enable the vehicle for reach a specific location.

## Project Structure

The project is structured as follows:

- `scripts/`: Contains scripts for running the project.
- `launch/`: Contains launch files for starting the project.
- `Model/`: Contains the yolov5 models for detect and classificate.

## Languages & Libraries Used

The project is primarily written in Python. The following Python libraries are used by the scripts:

- `rospy`: A Python library for ROS.
- `cv2`: A Python library for computer vision.
- `numpy`: A Python library for numerical computing.
- `cv_bridge`: A Python library for converting between ROS Image messages and OpenCV images.

## Messages
ROS uses a message-passing architecture, so the project also uses default message definitions.
The following messages are used in the project:

- `sensor_msgs/Image`: A message type for publishing images from a camera.
- `geometry_msgs/Point`: A message type for publishing detected object coordinates.
- `std_msgs/String`: A message type for publishing object classification results.
- `std_msgs/Float64`: A message type for publishing a Joint angle of the Robotic Arm.

## Scripts

The scripts are divided in 2 folders, the `classes/` are used and imported in the scripts inside the `nodes/` folder.

The following scripts are included in the project:

- `classes/`: A folder containing the following Python classes:
    - `camera.py`: A Python script defining the Camera class.
    - `inverseKinematics.py`: A Python script defining the InverseKinematics class.
    - `modelPredict.py`: A Python script defining the ModelPredict class.

- `nodes/`: A folder containing the following Python nodes:
    - `armMovement.py`: A Python script for controlling the robotic arm movement.
    - `cameraImage.py`: A Python script for publishing a camera image.
    - `objectClassificator.py`: A Python script for subscribing to a camera image and classifying objects.

## Launch

To run the project, you can use the following command:

`roslaunch JetAuto2 jetauto.launch`

This will start the project using the `jetauto.launch` launch file.

## License

This project is licensed under the BSD 3-Clause License. See the LICENSE file for details.