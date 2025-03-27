#!/bin/bash

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /home/root/ros2_ws/install/setup.bash
source /home/root/uros_ws/install/setup.bash
exec "$@"
