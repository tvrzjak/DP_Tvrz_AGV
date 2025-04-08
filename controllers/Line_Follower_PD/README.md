# Line_Follower_PD_controller

## Overview
This ROS 2 package implements a PD (Proportional-Derivative) controller for line-following robots. It subscribes to line center data and QR code messages to adjust the robot's velocity and LED indicators, publishing velocity commands to control the robot's movement.

## Features
- Uses a PD controller to adjust angular velocity based on the line center position.
- Responds to QR codes to adjust linear speed and set LED colors.
- Publishes `geometry_msgs/msg/Twist` for robot control and `std_msgs/msg/ColorRGBA` for LED feedback.

## Dependencies
- ROS 2 (tested with Humble/Jazzy)
- Python 3
- `rclpy`
- `geometry_msgs`
- `std_msgs`

### Required Sensor Drivers
This package relies on data from two separate sensor drivers:
1. **`SW_OGS_ros2_driver`**:
   - Provides line center data via the `/line_center` topic (`std_msgs/msg/Float32`).
   - agusure that this package is installed and running to provide line position data.
2. **`SW_DCR_ros2_driver`**:
   - Provides QR code data via the `/qr_code` topic (`std_msgs/msg/String`).
   - Ensure this package is installed and running to provide QR code inputs.

Both drivers must be installed and operational in your ROS 2 workspace for this controller to function correctly. See their respective repositories for installation instructions.

## Installation
1. Clone this repository (or only this folder) into your ROS 2 workspace `src` directory:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/tvrzjak/DP_Tvrz_AGV.git