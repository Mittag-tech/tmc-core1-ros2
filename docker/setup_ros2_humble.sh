#!/bin/bash

source /opt/ros/humble/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source /home/root/ros2_ws/install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source /home/root/ros2_ws/install/setup.bash
exec "$@"
