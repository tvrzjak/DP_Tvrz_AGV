# Leuze AGV Navigation

This repository provides a Docker container setup for autonomous navigation using ROS 2 Humble and the Navigation2 (Nav2) stack. The container includes dependencies for navigation, visualization, and mapping, with example configurations for the Leuze AGV. Users can extend the setup by adding custom plugins or modifying parameters.

## Features

- **ROS 2 Humble Base**: Built on the official `ros:humble` Docker image.
- **Navigation2 Stack**: Includes Nav2 and its bringup package for autonomous navigation.
- **Visualization**: RViz2 and Joint State Publisher for visualizing robot states and environments.
- **Mapping**: Supports SLAM Toolbox for online mapping (e.g., `online_async_launch.py`).
- **Customizable**: Pre-configured with example map (`map.yaml`) and Nav2 parameters (`nav2_params_LeuzeAGV_humble_omni.yaml`), with support for custom plugins.
- **Persistent Workspace**: ROS 2 workspace at `/ros2_ws` with automatic sourcing of `/opt/ros/humble/setup.bash`.

## Prerequisites

- **Docker Engine**: Installed and running on the host system.
- **Docker Compose** (optional): For easier management of the container.

## Usage

### Build the Container

To build and run the container using Docker Compose:

```bash
docker compose up --build -d
```

Alternatively, to build the Docker image manually:

```bash
docker build -t leuze_agv_navigation .
```

### Run the Container

To start the container and launch the Nav2 stack with the provided example configuration:

```bash
docker run -it leuze_agv_navigation
```

This will:
- Source `/opt/ros/humble/setup.bash`.
- Launch the Nav2 bringup with the specified map (`map.yaml`) and parameters (`nav2_params_LeuzeAGV_humble_omni.yaml`).

To run SLAM Toolbox for online mapping:

```bash
docker exec -it leuze_agv_navigation bash -c "source /opt/ros/humble/setup.bash && ros2 run slam_toolbox online_async_launch.py"
```

### Access the Container

To access the container's bash shell:

```bash
docker exec -it leuze_agv_navigation bash
```

Inside the container, the ROS 2 environment is automatically sourced via `~/.bashrc`.

## Configuration

- **Map File**: The example uses `map.yaml` for the map. Replace it with your own map file if needed.
- **Nav2 Parameters**: The `nav2_params_LeuzeAGV_humble_omni.yaml` file configures the Nav2 stack for an omnidirectional AGV. Modify or replace it to suit your robot.
- **Custom Plugins**: Add custom Nav2 plugins by extending the workspace at `/ros2_ws` and rebuilding the container.

## Dockerfile Overview

The Dockerfile sets up the container with the following:

1. **Base Image**: `ros:humble`.
2. **Dependencies**:
   - `python3-colcon-common-extensions` and `python3-rosdep` for ROS 2 workspace management.
   - `ros-humble-joint-state-publisher` and `ros-humble-rviz2` for visualization.
   - `ros-humble-navigation2` and `ros-humble-nav2-bringup` for navigation.
3. **Workspace**: Creates a ROS 2 workspace at `/ros2_ws`.
4. **Environment**: Sources `/opt/ros/humble/setup.bash` in `~/.bashrc` for persistence.
5. **Default Command**: Launches the Nav2 bringup with the example map and parameters.

## Extending the Container

To add custom packages or plugins:

1. Copy your ROS 2 packages to `/ros2_ws/src` in the Dockerfile or mount them as a volume.
2. Rebuild the container with `docker compose up --build` or `docker build`.
3. Source the workspace and build your packages inside the container:

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Notes

- The container is designed for development and testing. For production, consider optimizing the image size and security settings.
- Ensure the map and parameter files are available in the container or mounted as volumes.
- For mapping, SLAM Toolbox can be run separately as shown above.

## Troubleshooting

- **Nav2 Fails to Start**: Verify that the map and parameter files exist and are correctly formatted.
- **SLAM Toolbox Issues**: Ensure the robot's sensor data is properly published to the expected topics.
- **Docker Permissions**: If you encounter permission issues, ensure your user is in the Docker group or run commands with `sudo`.