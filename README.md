# Intel RealSense T265 ROS2 Node

## Overview
This ROS2 package provides an interface for the **Intel RealSense T265** tracking camera using the **RealSense SDK (librealsense2)**. The node publishes IMU, odometry, and fisheye images while broadcasting TF transformations for easy visualization and integration into robotic applications.

## ROS2 TOPICS
- **Odometry publishing** (`/rs_t265/odom`): Provides camera position and orientation.
- **IMU data publishing** (`/rs_t265/imu`): Publishes accelerometer and gyroscope readings.
- **Fisheye camera streams** (`/rs_t265/fisheye_left`, `/rs_t265/fisheye_right`): Provides grayscale fisheye format images.

## Installation & Setup

### **1. Install Librealsense2**
Ensure you have the **Librealsense2 library** compatible with your system:

```bash
sudo apt install librealsense2-dev
```

### **2. Clone and Build the Package**
```bash
cd ~/ros2_ws/src
git clone https://github.com/RubenCasal/my_t265_node.git
cd ~/ros2_ws
colcon build --packages-select my_t265_node
source install/setup.bash
```

### **3. Running the Node**
To start the T265 node:
```bash
ros2 run my_t265_node t265_node
```

To verify the published topics:
```bash
ros2 topic list
```

To visualize in RViz2, launch RViz and add the **TF, Image, and Odometry** displays.

## Usage Examples
### **Check Published Odometry Data**
```bash
ros2 topic echo /rs_t265/odom
```
### **Check IMU Data**
```bash
ros2 topic echo /rs_t265/imu
```
### **Check Fisheye Image Stream**
```bash
ros2 topic hz /rs_t265/fisheye_left
```
### **Visualize in RViz2**
```bash
rviz2
```
- Set **Fixed Frame** to `odom`.
- Add **TF, Odometry, and Image** displays.



