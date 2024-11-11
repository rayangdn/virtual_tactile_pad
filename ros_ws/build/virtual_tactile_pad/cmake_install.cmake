# Install script for directory: /home/rayann/Desktop/virtual_tactile_pad/ros_ws/src/virtual_tactile_pad

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/install")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/home/rayann/miniconda3/envs/ros_env/bin/x86_64-conda-linux-gnu-objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/virtual_tactile_pad/msg" TYPE FILE FILES "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/src/virtual_tactile_pad/msg/ContactForce.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/virtual_tactile_pad/cmake" TYPE FILE FILES "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/build/virtual_tactile_pad/catkin_generated/installspace/virtual_tactile_pad-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/devel/include/virtual_tactile_pad")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/devel/share/roseus/ros/virtual_tactile_pad")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/devel/share/common-lisp/ros/virtual_tactile_pad")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/devel/share/gennodejs/ros/virtual_tactile_pad")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/rayann/miniconda3/envs/ros_env/bin/python3.11" -m compileall "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/devel/lib/python3.11/site-packages/virtual_tactile_pad")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages" TYPE DIRECTORY FILES "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/devel/lib/python3.11/site-packages/virtual_tactile_pad")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/build/virtual_tactile_pad/catkin_generated/installspace/virtual_tactile_pad.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/virtual_tactile_pad/cmake" TYPE FILE FILES "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/build/virtual_tactile_pad/catkin_generated/installspace/virtual_tactile_pad-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/virtual_tactile_pad/cmake" TYPE FILE FILES
    "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/build/virtual_tactile_pad/catkin_generated/installspace/virtual_tactile_padConfig.cmake"
    "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/build/virtual_tactile_pad/catkin_generated/installspace/virtual_tactile_padConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/virtual_tactile_pad" TYPE FILE FILES "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/src/virtual_tactile_pad/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/virtual_tactile_pad" TYPE PROGRAM FILES "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/build/virtual_tactile_pad/catkin_generated/installspace/ft_process_node.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/virtual_tactile_pad" TYPE PROGRAM FILES "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/build/virtual_tactile_pad/catkin_generated/installspace/visualize_node.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/virtual_tactile_pad" TYPE PROGRAM FILES "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/build/virtual_tactile_pad/catkin_generated/installspace/data_saver_node.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/virtual_tactile_pad" TYPE PROGRAM FILES "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/build/virtual_tactile_pad/catkin_generated/installspace/digits_recognition_node.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/virtual_tactile_pad" TYPE PROGRAM FILES "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/build/virtual_tactile_pad/catkin_generated/installspace/optitrack_calibration_node.py")
endif()

