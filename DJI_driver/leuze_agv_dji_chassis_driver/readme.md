# Leuze AGV DJI Chassis Driver

## Overview
This repository provides a dockerized and easily deployable version of the existing DJI Robomaster driver ([https://jeguzzi.github.io/robomaster_ros](https://jeguzzi.github.io/robomaster_ros)), originally developed for the DJI Robomaster platform. The driver has been slightly modified to better suit the needs of autonomous navigation, enhancing its compatibility with navigation stacks and real-world robotic applications. These changes build upon the foundational work by Jeguzzi, adapting it for seamless integration into Leuze AGV projects.

## Original Description
This repository contains a ROS package to communicate with the DJI Robomaster platform.

### Setup

#### Native ROS
**Prerequisites**  
- ROS (Humble) installed and workspace set up.

**Usage**  
1. Set up the Robomaster SDK as described in `robomaster_ros/README.md`.  
2. Build using Colcon:  
   ```bash
   colcon build
   ```  
3. Use `RM.sh` to launch the driver:  
   ```bash
   ./RM.sh
   ```

#### Docker
- Run detached (restart set to always):  
  ```bash
  docker compose up -d
  ```  
- Build and run detached:  
  ```bash
  docker compose up --build -d
  ```

## License
This package is released under the same license as the original [robomaster_ros](https://jeguzzi.github.io/robomaster_ros) project (refer to its documentation for details).