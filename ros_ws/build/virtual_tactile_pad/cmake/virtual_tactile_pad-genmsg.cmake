# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "virtual_tactile_pad: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ivirtual_tactile_pad:/home/rayann/Desktop/virtual_tactile_pad/ros_ws/src/virtual_tactile_pad/msg;-Igeometry_msgs:/home/rayann/miniconda3/envs/ros_env/share/geometry_msgs/cmake/../msg;-Ivisualization_msgs:/home/rayann/miniconda3/envs/ros_env/share/visualization_msgs/cmake/../msg;-Istd_msgs:/home/rayann/miniconda3/envs/ros_env/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(virtual_tactile_pad_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/src/virtual_tactile_pad/msg/ContactForce.msg" NAME_WE)
add_custom_target(_virtual_tactile_pad_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "virtual_tactile_pad" "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/src/virtual_tactile_pad/msg/ContactForce.msg" "std_msgs/Header:geometry_msgs/Vector3:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(virtual_tactile_pad
  "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/src/virtual_tactile_pad/msg/ContactForce.msg"
  "${MSG_I_FLAGS}"
  "/home/rayann/miniconda3/envs/ros_env/share/std_msgs/cmake/../msg/Header.msg;/home/rayann/miniconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/rayann/miniconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/virtual_tactile_pad
)

### Generating Services

### Generating Module File
_generate_module_cpp(virtual_tactile_pad
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/virtual_tactile_pad
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(virtual_tactile_pad_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(virtual_tactile_pad_generate_messages virtual_tactile_pad_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/src/virtual_tactile_pad/msg/ContactForce.msg" NAME_WE)
add_dependencies(virtual_tactile_pad_generate_messages_cpp _virtual_tactile_pad_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(virtual_tactile_pad_gencpp)
add_dependencies(virtual_tactile_pad_gencpp virtual_tactile_pad_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS virtual_tactile_pad_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(virtual_tactile_pad
  "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/src/virtual_tactile_pad/msg/ContactForce.msg"
  "${MSG_I_FLAGS}"
  "/home/rayann/miniconda3/envs/ros_env/share/std_msgs/cmake/../msg/Header.msg;/home/rayann/miniconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/rayann/miniconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/virtual_tactile_pad
)

### Generating Services

### Generating Module File
_generate_module_eus(virtual_tactile_pad
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/virtual_tactile_pad
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(virtual_tactile_pad_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(virtual_tactile_pad_generate_messages virtual_tactile_pad_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/src/virtual_tactile_pad/msg/ContactForce.msg" NAME_WE)
add_dependencies(virtual_tactile_pad_generate_messages_eus _virtual_tactile_pad_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(virtual_tactile_pad_geneus)
add_dependencies(virtual_tactile_pad_geneus virtual_tactile_pad_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS virtual_tactile_pad_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(virtual_tactile_pad
  "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/src/virtual_tactile_pad/msg/ContactForce.msg"
  "${MSG_I_FLAGS}"
  "/home/rayann/miniconda3/envs/ros_env/share/std_msgs/cmake/../msg/Header.msg;/home/rayann/miniconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/rayann/miniconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/virtual_tactile_pad
)

### Generating Services

### Generating Module File
_generate_module_lisp(virtual_tactile_pad
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/virtual_tactile_pad
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(virtual_tactile_pad_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(virtual_tactile_pad_generate_messages virtual_tactile_pad_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/src/virtual_tactile_pad/msg/ContactForce.msg" NAME_WE)
add_dependencies(virtual_tactile_pad_generate_messages_lisp _virtual_tactile_pad_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(virtual_tactile_pad_genlisp)
add_dependencies(virtual_tactile_pad_genlisp virtual_tactile_pad_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS virtual_tactile_pad_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(virtual_tactile_pad
  "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/src/virtual_tactile_pad/msg/ContactForce.msg"
  "${MSG_I_FLAGS}"
  "/home/rayann/miniconda3/envs/ros_env/share/std_msgs/cmake/../msg/Header.msg;/home/rayann/miniconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/rayann/miniconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/virtual_tactile_pad
)

### Generating Services

### Generating Module File
_generate_module_nodejs(virtual_tactile_pad
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/virtual_tactile_pad
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(virtual_tactile_pad_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(virtual_tactile_pad_generate_messages virtual_tactile_pad_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/src/virtual_tactile_pad/msg/ContactForce.msg" NAME_WE)
add_dependencies(virtual_tactile_pad_generate_messages_nodejs _virtual_tactile_pad_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(virtual_tactile_pad_gennodejs)
add_dependencies(virtual_tactile_pad_gennodejs virtual_tactile_pad_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS virtual_tactile_pad_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(virtual_tactile_pad
  "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/src/virtual_tactile_pad/msg/ContactForce.msg"
  "${MSG_I_FLAGS}"
  "/home/rayann/miniconda3/envs/ros_env/share/std_msgs/cmake/../msg/Header.msg;/home/rayann/miniconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/rayann/miniconda3/envs/ros_env/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/virtual_tactile_pad
)

### Generating Services

### Generating Module File
_generate_module_py(virtual_tactile_pad
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/virtual_tactile_pad
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(virtual_tactile_pad_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(virtual_tactile_pad_generate_messages virtual_tactile_pad_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rayann/Desktop/virtual_tactile_pad/ros_ws/src/virtual_tactile_pad/msg/ContactForce.msg" NAME_WE)
add_dependencies(virtual_tactile_pad_generate_messages_py _virtual_tactile_pad_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(virtual_tactile_pad_genpy)
add_dependencies(virtual_tactile_pad_genpy virtual_tactile_pad_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS virtual_tactile_pad_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/virtual_tactile_pad)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/virtual_tactile_pad
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(virtual_tactile_pad_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET visualization_msgs_generate_messages_cpp)
  add_dependencies(virtual_tactile_pad_generate_messages_cpp visualization_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/virtual_tactile_pad)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/virtual_tactile_pad
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(virtual_tactile_pad_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET visualization_msgs_generate_messages_eus)
  add_dependencies(virtual_tactile_pad_generate_messages_eus visualization_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/virtual_tactile_pad)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/virtual_tactile_pad
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(virtual_tactile_pad_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET visualization_msgs_generate_messages_lisp)
  add_dependencies(virtual_tactile_pad_generate_messages_lisp visualization_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/virtual_tactile_pad)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/virtual_tactile_pad
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(virtual_tactile_pad_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET visualization_msgs_generate_messages_nodejs)
  add_dependencies(virtual_tactile_pad_generate_messages_nodejs visualization_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/virtual_tactile_pad)
  install(CODE "execute_process(COMMAND \"/home/rayann/miniconda3/envs/ros_env/bin/python3.11\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/virtual_tactile_pad\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/virtual_tactile_pad
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(virtual_tactile_pad_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET visualization_msgs_generate_messages_py)
  add_dependencies(virtual_tactile_pad_generate_messages_py visualization_msgs_generate_messages_py)
endif()
