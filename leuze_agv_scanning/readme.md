# Leuze AGV Scanning

This repository provides a Docker container for laser scanning with Leuze AGV, integrating ROS 2 Humble with a laser scan merger for combining multiple LIDAR scans. The setup is designed to merge scans from multiple LIDARs (typically placed at the corners of an AGV) into a single virtual scan for ROS 2 navigation, which supports only one LIDAR input. Users must provide their own LIDAR driver package to interface with their specific LIDAR hardware.

## Features

- **LIDAR Driver Placeholder**: Users must include their own LIDAR driver (e.g., similar to the Leuze RSL400 driver used as an example).
- **Laser Scan Merger**: Merges scans from two LIDARs into a single virtual scan for navigation.
- **ROS 2 Workspace**: Pre-configured workspace at `/ros2_ws` with sourced environment.
- **Configurable Parameters**: LIDAR IPs, ports, and relative orientations are set in `ros2_laser_scan_merger/config/params`.
- **Visualization and Utilities**: Includes RViz2, Joint State Publisher, and PCL conversions for point cloud processing.

## Purpose

The container facilitates laser scanning for AGV navigation by:
1. Allowing users to integrate their own LIDAR driver for specific hardware.
2. Merging scans from multiple LIDARs (e.g., placed at AGV corners) into a single scan, as ROS 2 navigation typically expects one LIDAR input.
3. Providing configuration for LIDAR parameters (IP addresses, ports, and relative orientations) for flexible deployment.

## Prerequisites

- **Docker Engine**: Installed and running on the host system.
- **Docker Compose** (optional): For easier container management.
- **LIDAR Driver**: You must provide a ROS 2-compatible driver package for your LIDAR hardware. For example, the Leuze RSL400 driver was used as a reference but is not included.

## Usage

### Configuration

1. **LIDAR Driver**:
   - Obtain or develop a ROS 2 driver package for your LIDAR hardware.
   - Place the driver package in `/ros2_ws/src` (e.g., by copying it into the Dockerfile or mounting it as a volume).
   - Example: The Leuze RSL400 driver was used with launch files for two LIDARs publishing to `scan1` and `scan2` topics. Replace with your driver's equivalent.
2. **Scanner Parameters**:
   - Configure the IP addresses, UDP ports, and relative orientations of the LIDARs in `ros2_laser_scan_merger/config/params`.
   - Example defaults (adjust for your LIDARs):
     - Scanner 1: `192.168.20.5:9991`
     - Scanner 2: `192.168.20.7:9992`
3. **Docker Compose (Optional)**:
   - Update `docker-compose.yml` with the correct LIDAR IPs and ports if using Docker Compose.

### Build and Run with Docker

1. Build and run the container in detached mode:

```bash
docker compose up --build -d
```

Alternatively, build the Docker image manually:

```bash
docker build -t leuze_agv_scanning .
```

Then run it:

```bash
docker run -it leuze_agv_scanning
```

**Note**: The container is configured in `docker-compose.yml` to start on machine boot and restart on error.

The default command:
- Launches two LIDAR drivers (replace with your driver's launch files, e.g., publishing to `scan1` and `scan2`).
- Runs the laser scan merger to combine scans into a single virtual scan.

### Access the Container

To access the container's bash shell:

```bash
docker exec -it leuze_agv_scanning bash
```

The ROS 2 environment is automatically sourced via `~/.bashrc`.

### Run Natively (Without Docker)

Use the provided convenience scripts in the repository, adapted for your LIDAR driver:

1. **Individual Startup**:
   - `RSL1.sh`: Sources and runs the driver for LIDAR 1 (replace with your driver's launch command, e.g., `scan1`).
   - `RSL2.sh`: Sources and runs the driver for LIDAR 2 (replace with your driver's launch command, e.g., `scan2`).
   - `merger.sh`: Sources and runs the laser scan merger.

2. **Common Startup**:
   - `run.sh`: Runs both LIDAR drivers and the merger together (update with your driver's commands).

Example (after updating scripts):

```bash
./run.sh
```

## Dockerfile Overview

The Dockerfile sets up the container with:

1. **Base Image**: `ros:humble`.
2. **Environment Variables**: Default IPs (`192.168.20.5`, `192.168.20.7`) and ports (`9991`, `9992`) for LIDARs (adjust as needed).
3. **Dependencies**:
   - `python3-colcon-common-extensions` and `python3-rosdep` for workspace management.
   - `ros-humble-pcl-conversions`, `ros-humble-xacro`, `ros-humble-angles` for point cloud and transformation support.
   - `ros-humble-joint-state-publisher` and `ros-humble-rviz2` for visualization.
4. **Workspace**: Creates `/ros2_ws/src` and copies:
   - `ros2_laser_scan_merger` (scan merger).
   - `pointcloud_to_laserscan` (point cloud to laser scan conversion).
   - **Note**: The LIDAR driver (e.g., `rsl400_ros2_navigation`) is not included. You must add your own driver package.
5. **Build**: Installs dependencies with `rosdep` and builds the workspace with `colcon`.
6. **Environment**: Sources `/ros2_ws/install/setup.bash` in `~/.bashrc`.
7. **Default Command**: Launches two LIDAR drivers (placeholder) and the scan merger. Replace the driver launch commands with your own.

## Extending the Container

To integrate your LIDAR driver or modify configurations:

1. Copy your LIDAR driver package to `/ros2_ws/src` in the Dockerfile or mount it as a volume.
2. Update `ros2_laser_scan_merger/config/params` for your LIDAR setup (IPs, ports, orientations).
3. Rebuild the container:

```bash
docker compose up --build
```

4. Inside the container, build the workspace:

```bash
source /ros2_ws/install/setup.bash
colcon build
source install/setup.bash
```

## Notes

- **LIDAR Driver**: You must provide a ROS 2-compatible driver for your LIDAR. Ensure it publishes scans to topics compatible with the merger (e.g., `scan1`, `scan2`).
- **LIDAR Placement**: The merger assumes LIDARs are placed at AGV corners. Configure relative orientations in `ros2_laser_scan_merger/config/params`.
- **Navigation Integration**: The merged scan is suitable for ROS 2 navigation stacks (e.g., Nav2), which expect a single LIDAR input.
- **Production Use**: Optimize the image size and security settings for production environments.
- **File Access**: Mount configuration files or driver packages as volumes if they need frequent updates.

## Troubleshooting

- **LIDAR Not Detected**: Verify IP addresses, ports, and driver compatibility in `docker-compose.yml` or environment variables.
- **Merger Issues**: Check `ros2_laser_scan_merger/config/params` for correct LIDAR orientations and topic names.
- **Driver Integration**: Ensure your LIDAR driver publishes to the expected topics (`scan1`, `scan2`) and is built correctly in the workspace.
- **Docker Permissions**: Ensure your user is in the Docker group or use `sudo` for Docker commands.
- **ROS Topics**: Confirm that scan topics are published using `ros2 topic list`.