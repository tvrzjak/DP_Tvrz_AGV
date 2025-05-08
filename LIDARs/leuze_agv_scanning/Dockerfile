FROM ros:humble

# Set environment variables with default values
ENV SCANNING_IP_1=192.168.20.5 \
    SCANNING_IP_2=192.168.20.7 \
    SCANNING_PORT_1=9991 \
    SCANNING_PORT_2=9992

# Ensure ROS 2 repository is properly configured before installing dependencies
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-humble-pcl-conversions \
        ros-humble-joint-state-publisher \
        ros-humble-rviz2 \
        ros-humble-xacro \
        ros-humble-angles && \
    rm -rf /var/lib/apt/lists/*

# Update rosdep (without re-initializing)
RUN rosdep update

# Create the ROS2 workspace
RUN mkdir -p /ros2_ws/src

# Copy the source code into the workspace
COPY ros2_laser_scan_merger /ros2_ws/src/ros2_laser_scan_merger
COPY rsl400_ros2_navigation /ros2_ws/src/rsl400_ros2_navigation
COPY pointcloud_to_laserscan /ros2_ws/src/pointcloud_to_laserscan

# Set the working directory
WORKDIR /ros2_ws

# Install missing dependencies using rosdep
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y"

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"

# Add setup sourcing to .bashrc for persistence
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set the default command with sourced environment
CMD ["/bin/bash", "-c", " \
    (source /ros2_ws/install/setup.bash && ros2 launch leuze_bringup leuze_bringup_rsl400.launch.py sensor_ip:=192.168.20.5 port:=9991 topic:=scan1) & \
    (source /ros2_ws/install/setup.bash && ros2 launch leuze_bringup leuze_bringup_rsl400.launch.py sensor_ip:=192.168.20.7 port:=9992 topic:=scan2) & \
    (source /ros2_ws/install/setup.bash && ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py) && \
    wait"]
