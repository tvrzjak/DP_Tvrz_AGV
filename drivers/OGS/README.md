# SW_OGS_ros2_driver

## Overview
This ROS 2 package provides a driver for Leuze OGS sensors (line reader), designed to process serial data and publish the calculated line center as a `std_msgs/msg/Float32` message. The driver is implemented in Python and is compatible with ROS2.

## Features
- Receives raw serial data from Leuze OGS sensors via UART.
- Calculates the center of a detected line based on left and right edge positions.
- Publishes the line center as a `Float32` message.

## Dependencies
- ROS2 (tested with Humble)
- Python 3
- `rclpy`
- `std_msgs`
- `pyserial`

## Installation
1. Clone this repository (or only this folder) into your ROS 2 workspace `src` directory:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/tvrzjak/DP_Tvrz_AGV.git



