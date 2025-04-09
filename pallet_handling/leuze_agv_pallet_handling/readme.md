
# Pallet Detection ROS 2 Package

## Overview
This ROS 2 package provides a set of nodes for detecting pallets using LIDAR data, processing the detections to identify pallet structures, and transforming their coordinates into a map frame for navigation purposes. It is designed for real-world robotic applications, such as autonomous docking under detected pallets.

The package includes three main scripts and a supporting user input node, each handling a specific part of the pallet detection and docking pipeline.

## Scripts

### 1. `dbscan_object_detection.py`
- **Purpose**: Performs real-time object detection directly from LIDAR data using DBSCAN clustering.
- **Input**: LIDAR data (`sensor_msgs/LaserScan`) from the `/scan` topic.
- **Output**: Publishes clusters as `visualization_msgs/MarkerArray` on the `/dbscan_groups` topic.
- **Details**: Clusters LIDAR points into groups based on proximity (configurable `eps` radius) and filters them by size. It is easily testable in simulation with any LIDAR-equipped robot.

### 2. `pallet_detection.py`
- **Purpose**: Processes detected clusters to identify pallets based on their spatial arrangement.
- **Input**: 
  - Clusters from `/dbscan_groups` (`visualization_msgs/MarkerArray`).
  - Global costmap (`nav_msgs/OccupancyGrid`) from `/global_costmap/costmap`.
  - Pallet in map frame (`geometry_msgs/PolygonStamped`) from `/pallet`.
  - User response (`std_msgs/Bool`) from `/user_response`.
- **Output**: 
  - Detected pallet in LIDAR frame (`geometry_msgs/PolygonStamped`) on `/pallet_help`.
  - Pallet center in LIDAR frame (`geometry_msgs/PointStamped`) on `/detected_center`.
  - Potential navigation goals (`visualization_msgs/MarkerArray`) on `/potential_goals`.
  - Goal pose for navigation (`geometry_msgs/PoseStamped`) on `/goal_pose`.
- **Details**: Analyzes cluster centers to find four points forming a rectangle matching pallet leg positions. It then computes feasible docking points and triggers navigation.

### 3. `pallet_transformer.py`
- **Purpose**: Transforms pallet data from the LIDAR frame to the map frame for absolute positioning.
- **Input**: 
  - Pallet center in LIDAR frame (`geometry_msgs/PointStamped`) from `/detected_center`.
  - Pallet rectangle in LIDAR frame (`geometry_msgs/PolygonStamped`) from `/pallet_help`.
- **Output**: 
  - Pallet center in map frame (`geometry_msgs/PointStamped`) on `/pallet_center`.
  - Pallet rectangle in map frame (`geometry_msgs/PolygonStamped`) on `/pallet`.
- **Details**: Uses TF2 to transform coordinates, ensuring the pallet's absolute position is known in the map frame for navigation and visualization.

### 4. `user_input.py`
- **Purpose**: Continuously prompts the user to confirm docking under a detected pallet.
- **Input**: User input via terminal (`y/n`).
- **Output**: Publishes user decision (`std_msgs/Bool`) on `/user_response`.
- **Details**: A simple node that waits for user confirmation to proceed with docking.

## Dependencies
- ROS 2 (tested with Humble/Jazzy)
- Python 3
- `rclpy`, `numpy`, `sklearn`, `cv2`, `tf2_ros`, `nav2_msgs`
- Install additional Python packages:
  ```bash
  pip install numpy scikit-learn opencv-python
  ```

## Installation and Running with Dockerfile
A `Dockerfile` is provided to build and run this package on any platform:
1. Build the Docker image:
   ```bash
   docker build -t pallet_detection .
   ```
2. Run the container (ensure ROS 2 environment and LIDAR data are available):
   ```bash
   docker run -it --net=host pallet_detection
   ```

## Running Individual Nodes
You can test individual nodes without Docker using `ros2 run`. For example:
- Object detection with clustering:
  ```bash
  ros2 run load_detection dbscan_object_detection
  ```
- Pallet detection:
  ```bash
  ros2 run load_detection pallet_detection
  ```
- Pallet transformation:
  ```bash
  ros2 run load_detection pallet_transformer
  ```
- User input:
  ```bash
  ros2 run load_detection user_input
  ```

This modular approach allows easy testing of specific components, such as clustering, in a LIDAR-equipped simulation.

## License
This package is released under the Apache-2.0 License.
