# JetAuto2

JetAuto2 is a ROS (Robot Operating System) project for controlling the Hiwonder JetAuto Pro Vehicle. It is designed to enable the vehicle to move autonomously, using SLAM for mapping and obstacle avoidance. Additionally, the vehicle is equipped with a built-in robotic arm that can be used to grab objects and this arm is controlled by an AI model that detects and classifies objects. Finally, planning and navegation algorithms are also used in order to enable the vehicle for reach a specific location.

## Project Structure

The project is structured as follows:

- `config/`: Contains yaml files with parameters for the nodes.
- `launch/`: Contains launch files for starting the project.
- `Model/`: Contains the yolov5 model for detection and classification.
- `scripts/`: Contains scripts for running the project.

## Dependencies

### Languages & Libraries Used

The project is primarily written in Python. The following Python libraries are used by the scripts:

- `cv2`: A Python library for computer vision.
- `cv_bridge`: A Python library for converting between ROS Image messages and OpenCV images.
- `numpy`: A Python library for numerical computing.
- `onnxruntime`: A Python library for infering with an ONNX model using either CPU or GPU.
- `rospy`: A Python library for ROS.

### Messages
ROS uses a message-passing architecture, so the project also uses default message definitions.
The following messages are used in the project:

- `geometry_msgs/Point`: A message type for publishing detected object coordinates.
- `geometry_msgs/Twist`: A message type for publishing the velocities (linear & angular) to move the vehicle.
- `hiwonder_servo_msgs/CommandDuration`: A message type for publishing a Joint angle and duration of the movement, for the Robotic Arm.
- `nav_msgs/OccupancyGrid`: A message type for publishing the SLAM map data.
- `sensor_msgs/JointState`: A message type for getting the current positions (angles) from the Robotic Arm joints.
- `sensor_msgs/Image`: A message type for publishing images from a camera.
- `sensor_msgs/LaserScan`: A message type for publishing Lidar Scan data.
- `std_msgs/String`: A message type for publishing object classification results.
- `std_msgs/Float64`: A message type for publishing the angle commands to the Robotic Arm joints.

## Configuration files

The provided configuration files are the following:

- `jetautoArm.yaml`: Contains all the modificable paramters for using the robotic Arm nodes (movement and trajectory).
- `jetautoClassification.yaml`: Contains the parameters for infering with the ONNX model.
- `jetautoObstacles.yaml`: Contains the parameters for performing SLAM and avoiding obstacles with the LiDAR.

## Scripts

The following ROS nodes are included in the project:

- `armMovement.py`: A Python script for controlling the robotic arm movement.
- `armTrajectory.py`: A Python script for controlling the robotic arm by interpolating a trajectory.
- `lidarAvoidance.py`: A Python script for using the LiDAR data and control the vehicle velocity to avoid obstacles.
- `objectClassificator.py`: A Python script for subscribing to a camera image and classifying objects.

The `Classes/` folder contains the following Python classes that are used and imported in the ROS nodes:
- `modelPredict.py`: A Python script defining the ModelPredict class with the input preprocessing and output postprocessing for the model inferences.
- `obstacleAvoidance.py`: A Python script defining the obstacleAvoidance class with the LiDAR data processing and algorithm for controlling the vehicle velocity.
- `robotArm.py`: A Python script defining the robotArm class with the IK solution and other functionalities.

## Build

### Installation

To install this package, clone the repository into your catkin workspace and build it using catkin.
```bash
cd ~/catkin_ws/src
git clone https://github.com/OctavioRguez/JetAuto2.git
mv ./JetAuto2 ./jet_auto2
cd ..
catkin_make
```

### Launch

- To run the roboticArm controller, you can use the command:  
```bash
roslaunch jet_auto2 roboticArm.launch
```

- To run the detection and classification model, you can use the command:     
```bash
roslaunch jet_auto2 modelClassification.launch
```

- To run the SLAM mapping and obstacle avoidance, you can use the command:     
```bash
roslaunch jet_auto2 slam.launch
```

- Finally, to run the whole project, you can use the following command (This will start all the launchs provided by the package):     
```bash
roslaunch jet_auto2 jetauto.launch
```   

## License

This project is licensed under the BSD 3-Clause License. See the [LICENSE](LICENSE) file for details.