#!/bin/bash

source /opt/ros/jazzy/setup.bash
colcon build
source /home/root/ros2_ws/install/setup.bash
exec "$@"