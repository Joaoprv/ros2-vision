# ROS2 Object Detection & Pose Estimation on Raspberry Pi 5

This project explores real-time object detection and 3D pose estimation using an off-the-shelf RGB camera with **ROS2** on a **Raspberry Pi 5**. It is part of a broader effort to build low-cost, scalable robotics perception systems.

## Project Goals

- Implement object detection using a lightweight model.
- Estimate the 3D pose of detected objects relative to the camera.
- Deploy and optimize the pipeline on a Raspberry Pi 5 running ROS2.
- Future milestone: Integrate LIDAR and perform sensor fusion for enhanced spatial perception.

## Features

- Real-time object detection using OpenCV + pre-trained models
- ROS2 integration with custom nodes for image streaming and inference
- 3D pose estimation using homography and camera calibration
- Modular codebase for future expansion (e.g., depth sensing, SLAM)

## Technologies Used

- **Hardware**: Raspberry Pi 5, USB RGB camera
- **Software**: ROS2 Humble, OpenCV, Python, PyTorch (or ONNX)
- **Models**: -

## Setup Instructions

1. Clone this repository to your Raspberry Pi:
    ```bash
    git clone https://github.com/Joaoprv/ros2-vision.git
    cd ros2-vision
    ```