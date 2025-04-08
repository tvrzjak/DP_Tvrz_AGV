# SW_DCR_ros2_driver

## Overview
This ROS 2 package provides a driver for a DCR QR code scanner, designed to read serial data and publish detected QR codes as `std_msgs/msg/String` messages. The driver is implemented in Python and is compatible with ROS2.

## Features
- Receives raw serial data from a DCR QR code scanner via UART.
- Filters and publishes specific QR codes (e.g., "C01G", "C01m", "C01C", "C01X", "C01Y", "C01Z").
- Publishes valid codes to a ROS 2 topic.

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