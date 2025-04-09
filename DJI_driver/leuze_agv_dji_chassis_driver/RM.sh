#!/bin/bash

source /opt/ros/humble/setup.bash;
source install/local_setup.bash;
ros2 launch robomaster_ros main.launch model:=ep conn_type:=rndis