FROM ros:humble

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
        ros-humble-nav2-msgs \
        ros-humble-angles && \
    apt-get install -y python3-colcon-common-extensions python3-pip && \
    pip3 install scikit-learn && \
    pip3 install numpy && \
    pip3 install opencv-python && \
    rm -rf /var/lib/apt/lists/*

# Update rosdep (without re-initializing)
RUN rosdep update

# Create the ROS2 workspace
RUN mkdir -p /ros2_ws/src

# Copy the source code into the workspace
COPY load_detection /ros2_ws/src/load_detection

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
    (source /ros2_ws/install/setup.bash && ros2 run load_detection dbscan_object_detection) & \
    (source /ros2_ws/install/setup.bash && ros2 run load_detection pallet_detection) & \
    (source /ros2_ws/install/setup.bash && ros2 run load_detection pallet_transformer) && \
    wait"]
