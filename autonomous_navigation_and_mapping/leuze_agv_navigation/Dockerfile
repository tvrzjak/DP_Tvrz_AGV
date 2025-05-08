FROM ros:humble

# Install necessary dependencies
RUN apt-get update && \
    apt-get install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-humble-joint-state-publisher \
        ros-humble-rviz2 \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup && \
    rm -rf /var/lib/apt/lists/*

# Create the ROS2 workspace
RUN mkdir -p /ros2_ws

# Set the working directory
WORKDIR /ros2_ws

# Ensure all subsequent RUN commands use bash
SHELL ["/bin/bash", "-c"]

# Add setup sourcing to .bashrc for persistence
# Add setup sourcing to .bashrc for persistence using /opt/humble
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# CMD: Launch bash, source the /opt/humble setup file, and run a ROS2 node (e.g. rviz2)
CMD ["/bin/bash", "-c", " \
    (source /opt/ros/humble/setup.bash && ros2 launch nav2_bringup bringup_launch.py map:=map.yaml params_file:=nav2_params_LeuzeAGV_humble_omni.yaml) & \
    #(source /opt/ros/humble/setup.bash && ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=map.yaml) & \
    #(source /opt/ros/humble/setup.bash && ros2 run nav2_util lifecycle_bringup map_server) && \
    wait"]
