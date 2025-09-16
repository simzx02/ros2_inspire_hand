#!/bin/bash

# Purpose: Launch ROS 2 pick object force node

# -------------------------------
# Step 1: Source ROS 2 environls

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
echo "Launching..."

tmux new-session -d -s inspire "ros2 run inspire_hand_ros pick_object_node_logger"
tmux split-window -h "ros2 run inspire_hand_ros inspire_hand_visualizer_node"
tmux split-window -v "ros2 run inspire_hand_ros headless_driver_node"
tmux attach -t inspire

# Optional: print message after exit
echo "Exited."

    #    'dds_publisher_node = inspire_hand_ros.dds_publisher_node:main',
    #    'inspire_hand_visualizer_node = inspire_hand_ros.inspire_hand_visualizer_node:main',
    #    'headless_driver_node = inspire_hand_ros.headless_driver_node:main',
    #    'finger_control_node = inspire_hand_ros.finger_control_node:main',
    #    'pick_object_node = inspire_hand_ros.pick_object_node:main',
