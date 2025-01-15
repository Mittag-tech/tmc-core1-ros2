#!/bin/bash

source /opt/ros/humble/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source /home/root/ros2_ws/install/setup.bash
exec "$@"
