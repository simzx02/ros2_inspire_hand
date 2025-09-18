# Install script for directory: /home/user_dell02/simzx_ws/ros1_inspire_hand/src/ros1_inspire_hand

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/user_dell02/simzx_ws/ros1_inspire_hand/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/user_dell02/simzx_ws/ros1_inspire_hand/build/ros1_inspire_hand/catkin_generated/installspace/ros1_inspire_hand.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros1_inspire_hand/cmake" TYPE FILE FILES
    "/home/user_dell02/simzx_ws/ros1_inspire_hand/build/ros1_inspire_hand/catkin_generated/installspace/ros1_inspire_handConfig.cmake"
    "/home/user_dell02/simzx_ws/ros1_inspire_hand/build/ros1_inspire_hand/catkin_generated/installspace/ros1_inspire_handConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ros1_inspire_hand" TYPE FILE FILES "/home/user_dell02/simzx_ws/ros1_inspire_hand/src/ros1_inspire_hand/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ros1_inspire_hand" TYPE PROGRAM FILES
    "/home/user_dell02/simzx_ws/ros1_inspire_hand/src/ros1_inspire_hand/scripts/driver_ros1.py"
    "/home/user_dell02/simzx_ws/ros1_inspire_hand/src/ros1_inspire_hand/scripts/pick_object_node_pid_ros1.py"
    "/home/user_dell02/simzx_ws/ros1_inspire_hand/src/ros1_inspire_hand/scripts/visualizer_ros1.py"
    "/home/user_dell02/simzx_ws/ros1_inspire_hand/src/ros1_inspire_hand/scripts/open_finger.py"
    "/home/user_dell02/simzx_ws/ros1_inspire_hand/src/ros1_inspire_hand/scripts/pick_object_node_pid_ros1_open_close.py"
    )
endif()

