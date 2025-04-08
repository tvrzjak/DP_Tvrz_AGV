# Pure Pursuit Controller

## Overview
This ROS 2 package implements a Pure Pursuit algorithm for trajectory following in a simulated differential drive robot. It serves as a demonstration example, showcasing how to control a robot along a predefined path using the Pure Pursuit method. The package subscribes to the robot's pose via TF2 and publishes velocity commands to `/cmd_vel` while also publishing the planned trajectory to `/plan`.

## Features
- Implements the Pure Pursuit algorithm for smooth trajectory following.
- Dynamically adjusts linear and angular velocities based on the lookahead distance and heading error.
- Supports looping trajectories by appending the original path when the end is reached.
- Publishes the planned path for visualization in RViz.

## Dependencies
- ROS 2 (tested with Humble/Jazzy)
- Python 3
- `rclpy`
- `geometry_msgs`
- `nav_msgs`
- `tf2_ros`
- `numpy`

## Prerequisites for Simulation
To run this package in a simulation environment, you need an existing simulation package (e.g., `diff_robot_gazebo`). The following steps are required to set up and launch the simulation:

1. **Launch the Gazebo Simulation**:
   - Start the simulation with the robot spawned in Gazebo:
     ```bash
     ros2 launch diff_robot_gazebo spawn_robot.launch.py
     ```
   - This spawns the robot in a Gazebo world and provides the necessary TF2 transforms (e.g., `odom` to `base_footprint`).

2. **Run RViz2 for Visualization**:
   - Launch RViz2 to visualize the robot and its trajectory:
     ```bash
     rviz2 rviz2
     ```
   - Set the **Fixed Frame** to `odom` in RViz2 to align the visualization with the robot's odometry frame.

3. **Run the Regulator Node**:
   - Start the regulator node from the `diff_robot_gazebo` package to handle low-level control:
     ```bash
     ros2 run diff_robot_gazebo regulator
     ```

These components together provide the simulation environment needed to test the Pure Pursuit controller.

## Installation
1. Clone this repository into your ROS 2 workspace `src` directory:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```
2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select pure_pursuit_controller
   ```
3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage
### Running the Controller
- Launch the Pure Pursuit controller node:
  ```bash
  ros2 run pure_pursuit_controller trajectory_follower
  ```
- Alternatively, create a launch file (recommended) to start it alongside the simulation components.

### Published Topics
- `/cmd_vel`: `geometry_msgs/msg/Twist`
  - Velocity commands (linear.x and angular.z) for the robot.
- `/plan`: `nav_msgs/msg/Path`
  - The planned trajectory for visualization.

### Subscribed Topics
- None directly, but relies on TF2 transforms between `odom` and `base_footprint`.

## Notes on the Example
This package is a demonstration example focusing on the Pure Pursuit algorithm implementation. The predefined trajectory is hardcoded in the script, but users can modify it or replace it with a custom trajectory (e.g., loaded from a file) by adjusting the `points` list in `trajectory_follower.py`.

## Optional Localization in a Map
If users wish to integrate map-based localization, the simulation is prepared to support it since the robot includes a LIDAR. Here’s how to set it up:

1. **Add a Room Model to Gazebo**:
   - Place a room model in the Gazebo world by editing the world file in the `diff_robot_gazebo/world` directory or creating a custom `.world` file with obstacles.

2. **Create or Map the Environment**:
   - Use the SLAM Toolbox to generate a map:
     ```bash
     ros2 launch slam_toolbox online_async_launch.py
     ```
   - Drive the robot around (manually or via teleop) to map the environment, then save the map using:
     ```bash
     ros2 run nav2_map_server map_saver_cli -f my_map
     ```

3. **Launch Localization**:
   - Start the Nav2 localization stack with your map:
     ```bash
     ros2 launch nav2_bringup localization_launch.py map:=/path/to/my_map.yaml
     ```

4. **Adjust RViz2**:
   - Change the **Fixed Frame** in RViz2 to `map` to visualize the robot in the mapped frame.

5. **Custom Trajectory**:
   - Replace the hardcoded `points` in `trajectory_follower.py` with your own trajectory (e.g., loaded from a JSON file or generated dynamically).

The simulation supports this setup, and the robot’s LIDAR provides the necessary data for SLAM and localization.

## Configuration
- **Lookahead Distance**: Adjust the `lookAheadDis` parameter (default: 0.3 m) in the `control_loop` method to change how far ahead the robot looks.
- **Speed Limits**: Modify `max_speed`, `min_speed`, and `max_turn_vel` in `control_loop` to tune the robot’s behavior.
- **Trajectory**: Edit the `points` list in `__init__` or implement a method to load a custom trajectory.

## License
This package is released under the Apache-2.0 License.