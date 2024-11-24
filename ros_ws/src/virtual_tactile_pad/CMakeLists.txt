cmake_minimum_required(VERSION 3.0.2)
project(virtual_tactile_pad)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  visualization_msgs
  franka_msgs
  message_generation
)

# Add message files
add_message_files(
  FILES
  ContactForce.msg
)

# Generate messages
generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  franka_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs visualization_msgs  franka_msgs message_runtime 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/panda_process_node.py
  src/ft_process_node.py
  src/visualize_node.py
  src/data_saver_node.py
  src/digits_recognition_node.py
  src/optitrack_calibration_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
