# GPS Waypoint Navigation Project  

This repository contains the main codebase deployed on the AgileX Scout Mini robot for the ELTE AUS/IFRoS lab in 2024.  

The objective of this project is to enable the robot to autonomously navigate to a series of GPS-defined waypoints in an outdoor environment. At each waypoint, the robot captures an image of the surroundings. The system integrates GPS and IMU data for navigation and provides real-time visualization in RViz.

Created by:  
**Liviu Florescu**

**Lisa Paul Magoti**

**Selin Yavuz**

---

## Project Team & Responsibilities  

| **Name**             | **Responsibilities**               |  
|-----------------------|-------------------------------------|  
| **Liviu Florescu**       | GPS navigation, IMU integration, and image capture |  
| **Lisa Paul Magoti**       | Controller, image handling |  
| **Selin Yavuz**       | ROS topics and system architecture, debugging, documentation |  

---

## Architecture  


### Project Overview
The navigation system is designed to autonomously navigate a robot using GPS-defined waypoints. GPS coordinates are converted into a local coordinate frame for real-time navigation. The robot also captures images at each waypoint for later use.

Key features include:

- **GPS Navigation**: 
  The system converts GPS coordinates to a local coordinate frame to plan the robot’s movements.
- **IMU Integration**: 
  IMU data provides heading corrections and orientation information for precise navigation.
- **Image Capture**: 
  The robot captures images upon reaching each waypoint using an onboard camera.
- **Real-time Visualization**: 
  RViz is used to visualize the robot's path, current position, and navigation progress.


## Setup  

### Prerequisites  
Ensure you have ROS installed and configured on the robot.  

1. **Clone the Repository**  
   ```bash  
   git clone https://github.com/IFRoS-ELTE/navigators.git

    ```
3. **Build the Workspace**  
    ```bash  
    catkin_make  
    ```

4. **Connect CANBUS**  
    ```bash  
    rosrun scout_bringup setup_can2usb.bash
    ```

5. **Run the Launch File**  
    ```bash  
    roslaunch scout_bringup scout_minimal.launch
    ```

6. **Start GPS and Magnetometer**  
    ```bash  
    roslaunch xsens_mti_driver xsens.launch  

    ```
7. **Start Camera**  
    ```bash  
    roslaunch realsense2_camera rs_camera.launch

    ```
---
## Running the Code 

"basic.py" has the overall code of the project. With this script, we are navigating between two GPS waypoints in an outdoor setting, and capture an image with the camera. A visualization of the Rviz can be found below.

## Visuals from the robot

![Robot Picture](robot_pic.jpeg)
![Robot Picture 2](pic_2.jpeg)
![Robot Picture 3](pic_3.jpeg)

# Future Improvements

Below are some of the potential improvements and features that could be added to the project in the future:

| **To-Do**                               | **Description**                                                                                  | **Priority**   |
|-----------------------------------------|--------------------------------------------------------------------------------------------------|----------------|
| **Dynamic Path Planning**               | Improve path planning to adapt dynamically to changes in the environment (e.g., moving obstacles). | Medium         |
| **Obstacle Detection**                  | Implement obstacle detection to enable the robot to avoid collisions while navigating.            | High           |
| **Localization and Mapping Refinement** | Improve localization accuracy and SLAM algorithm for better map generation and robot positioning. | Medium         |

## Troubleshooting  
- **Robot Not Reaching Waypoints**
If the robot seems unable to reach the waypoints, check the GPS signal and verify the IMU calibration. It might be necessary to recalibrate the IMU and ensure a stable GPS connection.

- **Camera Issues**
If the robot is unable to capture images at waypoints, check the camera connection and confirm the camera launch file is running without errors.




- **Support**  
  For further assistance, please open an issue on the GitHub repository or contact the project maintainers.

---


## Acknowledgments  

We would like to thank Professor Istenes and CLC employees for their support and resources throughout this project.
