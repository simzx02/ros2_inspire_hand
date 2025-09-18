cmake_minimum_required(VERSION 3.0.2)
project(ros1_inspire_hand)

# Find catkin and required ROS packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

################################################
# Catkin package configuration
################################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

# Include directories (if you have any header files)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
install(PROGRAMS
  scripts/driver_ros1.py
  scripts/pick_object_node_pid_ros1.py
  scripts/visualizer_ros1.py
  scripts/open_finger.py
  scripts/pick_object_node_pid_ros1_open_close.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Note:
# External Python dependencies (e.g., inspire_sdkpy, unitree_sdk2py) should be installed via pip.
# Document these in your README or requirements.txt, not in CMakeLists.txt.

