# Leuze AGV Description Packages

This repository contains two ROS 2 packages, `Leuze_diff_AGV_description` and `Leuze_omni_AGV_description`, designed as general-purpose drivers for AGV chassis with differential and mecanum wheel configurations, respectively. These packages provide forward and inverse kinematics, along with the necessary publications to enable integration with a ROS 2 navigation stack. They serve as templates or inspiration for developing custom chassis drivers but require additional implementation, such as a PLC driver for communication with Siemens PLCs.

## Features

- **Leuze_diff_AGV_description**:
  - Supports AGVs with **differential drive** chassis.
  - Implements **forward and inverse kinematics** for precise motion control.
  - Publishes required ROS 2 topics (e.g., odometry, transforms) for navigation stack compatibility.
  
- **Leuze_omni_AGV_description**:
  - Supports AGVs with **mecanum wheel** chassis for omnidirectional movement.
  - Includes a reference **kinematics implementation** (typically handled in a PLC).
  - Publishes results of kinematics computations (e.g., velocity commands, odometry) for navigation.
  - Provides a sample kinematics file as a reference for custom implementations.

- **Navigation Stack Integration**:
  - Both packages publish essential data (e.g., `/odom`, `/tf`) to support ROS 2 navigation stacks like Nav2.
  - Configurable parameters for tuning kinematics and chassis behavior.

## Limitations

- **No PLC Driver**:
  - These packages do not include a driver for communication with Siemens PLCs.
  - Users must develop their own PLC driver to interface with the AGV's control system.
- **Omnidirectional Kinematics**:
  - In the `Leuze_omni_AGV_description` package, kinematics is typically computed in the PLC. The package only publishes the results.
  - A sample kinematics file is included but must be adapted for specific hardware.
- **Template Nature**:
  - These packages are not plug-and-play. They provide a foundation for building custom chassis drivers tailored to specific AGV hardware and PLC setups.

## Purpose

The `Leuze_diff_AGV_description` and `Leuze_omni_AGV_description` packages are designed to:
1. Provide a starting point for developing ROS 2 drivers for differential and omnidirectional AGV chassis.
2. Offer pre-implemented kinematics and ROS 2 topic publications for navigation integration.
3. Serve as a reference or template for users building custom drivers, especially when interfacing with Siemens PLCs or similar control systems.

## Prerequisites

- **ROS 2 Humble**: The packages are developed for ROS 2 Humble.
- **Hardware**:
  - An AGV with a differential drive or mecanum wheel chassis.
  - A Siemens PLC (or equivalent) for low-level control (user must provide the PLC driver).
- **Dependencies**:
  - Standard ROS 2 packages (e.g., `geometry_msgs`, `nav_msgs`, `tf2_ros`).
  - Install dependencies using `rosdep` (see Setup below).

## Setup

1. **Clone the Repository**:
   ```bash
   git clone <repository_url>
   cd <repository_directory>
   ```

2. **Create a ROS 2 Workspace**:
   ```bash
   mkdir -p ~/ros2_ws/src
   mv Leuze_diff_AGV_description Leuze_omni_AGV_description ~/ros2_ws/src
   cd ~/ros2_ws
   ```

3. **Install Dependencies**:
   ```bash
   source /opt/ros/humble/setup.bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the Workspace**:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## Usage

### Leuze_diff_AGV_description example

1. **Configure Parameters**:
   - Edit source files or add configuration files (e.g., in `config/` or `params/`, not have been done yet) to match your AGV's specifications (wheelbase, wheel radius, etc.).
   - Specify topic names and frame IDs as needed.

2. **Launch the Driver**:
   ```bash
   ros2 launch Leuze_diff_AGV_description robot.launch.py
   ```
   - This launches the driver, publishing odometry and transforms for navigation.
   - Ensure your PLC driver publishes velocity commands to the expected topics.

3. **Integrate with Navigation**:
   - Use with a ROS 2 navigation stack (e.g., Nav2) by configuring the navigation stack to subscribe to the published `/odom` and `/tf` topics.


4. **Integrate with Navigation**:
   - Configure the ROS 2 navigation stack to use the published `/odom` and `/tf` topics for omnidirectional navigation.

## Extending the Packages

To adapt these packages for your AGV:

1. **Add a PLC Driver**:
   - Develop a ROS 2 node to communicate with your Siemens PLC (e.g., using S7 communication protocols or OPC UA).
   - Publish velocity commands or kinematics results to the topics expected by the driver packages.

2. **Customize Kinematics**:
   - For `Leuze_diff_AGV_description`, modify the kinematics implementation in the source code to match your AGV's differential drive model.
   - For `Leuze_omni_AGV_description`, adapt the provided kinematics file or implement custom logic in your PLC.

3. **Update Parameters**:
   - Adjust configuration files to match your AGV's physical properties and ROS 2 topic structure.

4. **Build and Test**:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch <package_name> <launch_file>
   ```

## Notes

- **PLC Integration**:
  - Without a PLC driver, these packages cannot directly control the AGV. You must implement communication with your controll PC which provides encoder data.
- **Kinematics in Omni Case**:
  - The `Leuze_omni_AGV_description` package assumes kinematics is handled in the PLC. The provided kinematics file is a reference and may require significant modification.
- **Navigation Stack**:
  - Ensure your navigation stack is configured for the appropriate chassis type (differential or omnidirectional).
  - For mecanum wheels, verify that the navigation stack supports holonomic motion.
- **Template Usage**:
  - These packages are designed as starting points. Expect to invest time in customizing the driver and integrating it with your AGV's control system.

## Troubleshooting

- **No Odometry/Transforms**:
  - Verify that your PLC driver is publishing to the correct topics.
  - Check configuration files for correct topic names and frame IDs.
- **Kinematics Errors**:
  - For differential drive, ensure wheel parameters (e.g., radius, wheelbase) are accurate.
  - For omnidirectional, validate the kinematics implementation in the PLC or the provided file.
- **Navigation Stack Issues**:
  - Confirm that the navigation stack is configured to match the AGV's motion model (differential or holonomic).
  - Use `ros2 topic echo` and `rviz2` to debug published topics (`/odom`, `/tf`).
- **Build Failures**:
  - Ensure all dependencies are installed (`rosdep install`).
  - Check for errors in the package's `CMakeLists.txt` or `package.xml`.