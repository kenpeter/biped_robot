#!/bin/bash
# This script ensures a clean environment for building the ROS2 workspace.

echo "--- Clearing potentially conflicting environment variables ---"
unset COLCON_PREFIX_PATH
unset AMENT_PREFIX_PATH
echo "--- Sourcing ROS2 Humble environment ---"
source /opt/ros/humble/setup.bash

echo "--- Starting colcon build ---"
colcon build