#!/bin/bash
# Filename: launch_hand.sh
# Purpose: Launch ROS 2 hand publisher

# -------------------------------
# Step 1: Source ROS 2 environment
# Adjust this path according to your ROS 2 installation
source /opt/ros/humble/setup.bash

# -------------------------------
# Step 2: Source your ROS 2 workspace (if any)
# Adjust this path to your workspace install/setup.bash
WORKSPACE_SETUP=~/workspace/inspire_hand/ros2_inspire_hand/install/setup.bash
if [ -f "$WORKSPACE_SETUP" ]; then
    source "$WORKSPACE_SETUP"
fi

# -------------------------------
# Step 3: Launch the hand publisher
echo "Launching hand_publisher..."
ros2 launch hand_bringup hand_publisher.launch.py

# Optional: print message after exit
echo "hand_publisher.launch.py exited."
