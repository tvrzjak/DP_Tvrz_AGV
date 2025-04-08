# Pure Pursuit Controller (ROS 2 Node)

## Overview
This ROS 2 package implements a Pure Pursuit algorithm as a fully functional ROS 2 node for trajectory following. It is designed for real-world robotic applications where a map is already created, and the robot can localize itself within it. The node receives a drawn trajectory (e.g., from an external application), converts it into a navigable path, and follows it using the Pure Pursuit algorithm. It includes features to start/stop driving and publishes the planned path and goal points for visualization and debugging.

The primary focus is on practical deployment, with the assumption that localization (e.g., via Nav2) is already operational. For a simpler, simulation-based example, refer to the `simulation` directory.

## Features
- Implements the Pure Pursuit algorithm for precise trajectory following.
- Subscribes to `/path` (SVG path string) to receive trajectories drawn from an external application (see diploma thesis documentation for details).
- Supports start/stop functionality via the `/driving` topic (`std_msgs/Bool`).
- Publishes the planned trajectory to `/plan` and the current goal point to `/goal_point` for visualization.
- Integrates with a pre-existing map, using map metadata (resolution, origin) to align the trajectory.
- Adjustable parameters for lookahead distance, speed, and turning behavior.

## Dependencies
- ROS 2 (tested with Humble/Jazzy)
- Python 3
- `rclpy`
- `geometry_msgs`
- `std_msgs`
- `nav_msgs`
- `tf2_ros`
- `numpy`
- `svg.path` (install via `pip install svg.path`)

## Assumptions
- A map is already created and available via the `/map_server/map` service (e.g., using Nav2 or SLAM Toolbox).
- The robot is localized in the map, providing TF2 transforms from `map` to `base_footprint`.
- An external application sends SVG-formatted paths to the `/path` topic (see diploma thesis documentation for the expected format).

## Installation
1. Clone this repository into your ROS 2 workspace `src` directory:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```
2. Install the `svg.path` Python package:
   ```bash
   pip install svg.path
   ```
3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select pure_pursuit_controller
   ```
4. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage
### Running the Node
- Launch the Pure Pursuit controller node:
  ```bash
  ros2 run pure_pursuit_controller trajectory_follower
  ```
- Ensure that the map server and localization (e.g., Nav2) are running beforehand.

### Subscribed Topics
- `/path`: `std_msgs/String`
  - SVG-formatted path string sent from an external application to define the trajectory.
- `/driving`: `std_msgs/Bool`
  - Boolean value to start (`true`) or stop (`false`) the robot's movement.
- `/fake_path`: `std_msgs/Bool`
  - Triggers a hardcoded test trajectory for debugging purposes.

### Published Topics
- `/cmd_vel`: `geometry_msgs/Twist`
  - Velocity commands (linear.x and angular.z) to control the robot.
- `/plan`: `nav_msgs/Path`
  - The planned trajectory in the `map` frame for visualization.
- `/goal_point`: `geometry_msgs/PointStamped`
  - The current lookahead goal point in the `map` frame.

### Services Used
- `/map_server/map`: `nav_msgs/GetMap`
  - Retrieves map metadata (resolution, origin, height) to align the trajectory with the map.

## Sending Trajectories
Trajectories are sent as SVG path strings to the `/path` topic. The node converts these into a series of points aligned with the map's coordinate system. For details on the SVG format and how to draw/send paths from an application, refer to the diploma thesis documentation.

Example command to send a test path (replace with actual SVG string):
```bash
ros2 topic pub /path std_msgs/String "data: 'M 10 10 L 20 20'"
```

## Starting and Stopping
- To start driving:
  ```bash
  ros2 topic pub /driving std_msgs/Bool "data: true" --once
  ```
- To stop driving:
  ```bash
  ros2 topic pub /driving std_msgs/Bool "data: false" --once
  ```

This is basically solved by the application

## Configuration
- **Lookahead Distance**: Adjust `lookAheadDis` (default: 0.4 m) in the `control_loop` method to tune how far ahead the robot looks.
- **Speed Limits**: Modify `max_speed` (0.3 m/s), `min_speed` (0.01 m/s), and `max_turn_vel` (0.5 rad/s) in `control_loop`.
- **Proportional Gain**: Change `Kp` (default: 0.55) in `pure_pursuit_step` to adjust turning responsiveness.
- **Map Resolution**: Currently hardcoded in `convert_to_array`; consider loading it dynamically from a YAML file for flexibility.

## Simulation Example
For a simpler, simulation-based demonstration of the Pure Pursuit algorithm without map dependency, see the `simulation` directory. It includes a basic setup using Gazebo and RViz, ideal for testing the algorithm's core principles.

## License
This package is released under the Apache-2.0 License.
