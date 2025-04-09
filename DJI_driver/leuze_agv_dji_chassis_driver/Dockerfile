FROM ros:humble

# Ensure ROS 2 repository is properly configured before installing dependencies
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-humble-xacro \
        ros-humble-launch-xml \
        ros-humble-cv-bridge \
        ros-humble-launch-testing-ament-cmake \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-joy \
        ros-humble-joy-teleop \
        ros-humble-joy-linux \
        libopus-dev \
        python3-pip && \
    rm -rf /var/lib/apt/lists/*

# Install Python dependencies for RoboMaster-SDK
RUN python3 -m pip install numpy==1.23.5 numpy-quaternion pyyaml

# Install the forked RoboMaster-SDK
RUN python3 -m pip install git+https://github.com/jeguzzi/RoboMaster-SDK.git
RUN python3 -m pip install "git+https://github.com/jeguzzi/RoboMaster-SDK.git#egg=libmedia_codec&subdirectory=lib/libmedia_codec"

# Update rosdep (without re-initializing)
RUN rosdep update

# Create the ROS2 workspace
RUN mkdir -p /ros2_ws/src

# Copy the source code into the workspace
COPY robomaster_ros /ros2_ws/src/robomaster_ros

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
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/local_setup.bash && ros2 launch robomaster_ros main.launch model:=ep conn_type:=rndis"]
