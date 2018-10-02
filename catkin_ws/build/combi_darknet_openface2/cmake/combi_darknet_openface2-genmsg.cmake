# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "combi_darknet_openface2: 3 messages, 0 services")

set(MSG_I_FLAGS "-Icombi_darknet_openface2:/root/catkin_ws/src/combi_darknet_openface2/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(combi_darknet_openface2_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg" NAME_WE)
add_custom_target(_combi_darknet_openface2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "combi_darknet_openface2" "/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg" "geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/Faces.msg" NAME_WE)
add_custom_target(_combi_darknet_openface2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "combi_darknet_openface2" "/root/catkin_ws/src/combi_darknet_openface2/msg/Faces.msg" "std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/Vector3:geometry_msgs/Pose:combi_darknet_openface2/Face"
)

get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/ActionUnit.msg" NAME_WE)
add_custom_target(_combi_darknet_openface2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "combi_darknet_openface2" "/root/catkin_ws/src/combi_darknet_openface2/msg/ActionUnit.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(combi_darknet_openface2
  "/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/combi_darknet_openface2
)
_generate_msg_cpp(combi_darknet_openface2
  "/root/catkin_ws/src/combi_darknet_openface2/msg/Faces.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/combi_darknet_openface2
)
_generate_msg_cpp(combi_darknet_openface2
  "/root/catkin_ws/src/combi_darknet_openface2/msg/ActionUnit.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/combi_darknet_openface2
)

### Generating Services

### Generating Module File
_generate_module_cpp(combi_darknet_openface2
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/combi_darknet_openface2
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(combi_darknet_openface2_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(combi_darknet_openface2_generate_messages combi_darknet_openface2_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg" NAME_WE)
add_dependencies(combi_darknet_openface2_generate_messages_cpp _combi_darknet_openface2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/Faces.msg" NAME_WE)
add_dependencies(combi_darknet_openface2_generate_messages_cpp _combi_darknet_openface2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/ActionUnit.msg" NAME_WE)
add_dependencies(combi_darknet_openface2_generate_messages_cpp _combi_darknet_openface2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(combi_darknet_openface2_gencpp)
add_dependencies(combi_darknet_openface2_gencpp combi_darknet_openface2_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS combi_darknet_openface2_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(combi_darknet_openface2
  "/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/combi_darknet_openface2
)
_generate_msg_eus(combi_darknet_openface2
  "/root/catkin_ws/src/combi_darknet_openface2/msg/Faces.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/combi_darknet_openface2
)
_generate_msg_eus(combi_darknet_openface2
  "/root/catkin_ws/src/combi_darknet_openface2/msg/ActionUnit.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/combi_darknet_openface2
)

### Generating Services

### Generating Module File
_generate_module_eus(combi_darknet_openface2
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/combi_darknet_openface2
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(combi_darknet_openface2_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(combi_darknet_openface2_generate_messages combi_darknet_openface2_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg" NAME_WE)
add_dependencies(combi_darknet_openface2_generate_messages_eus _combi_darknet_openface2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/Faces.msg" NAME_WE)
add_dependencies(combi_darknet_openface2_generate_messages_eus _combi_darknet_openface2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/ActionUnit.msg" NAME_WE)
add_dependencies(combi_darknet_openface2_generate_messages_eus _combi_darknet_openface2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(combi_darknet_openface2_geneus)
add_dependencies(combi_darknet_openface2_geneus combi_darknet_openface2_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS combi_darknet_openface2_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(combi_darknet_openface2
  "/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/combi_darknet_openface2
)
_generate_msg_lisp(combi_darknet_openface2
  "/root/catkin_ws/src/combi_darknet_openface2/msg/Faces.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/combi_darknet_openface2
)
_generate_msg_lisp(combi_darknet_openface2
  "/root/catkin_ws/src/combi_darknet_openface2/msg/ActionUnit.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/combi_darknet_openface2
)

### Generating Services

### Generating Module File
_generate_module_lisp(combi_darknet_openface2
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/combi_darknet_openface2
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(combi_darknet_openface2_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(combi_darknet_openface2_generate_messages combi_darknet_openface2_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg" NAME_WE)
add_dependencies(combi_darknet_openface2_generate_messages_lisp _combi_darknet_openface2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/Faces.msg" NAME_WE)
add_dependencies(combi_darknet_openface2_generate_messages_lisp _combi_darknet_openface2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/ActionUnit.msg" NAME_WE)
add_dependencies(combi_darknet_openface2_generate_messages_lisp _combi_darknet_openface2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(combi_darknet_openface2_genlisp)
add_dependencies(combi_darknet_openface2_genlisp combi_darknet_openface2_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS combi_darknet_openface2_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(combi_darknet_openface2
  "/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/combi_darknet_openface2
)
_generate_msg_nodejs(combi_darknet_openface2
  "/root/catkin_ws/src/combi_darknet_openface2/msg/Faces.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/combi_darknet_openface2
)
_generate_msg_nodejs(combi_darknet_openface2
  "/root/catkin_ws/src/combi_darknet_openface2/msg/ActionUnit.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/combi_darknet_openface2
)

### Generating Services

### Generating Module File
_generate_module_nodejs(combi_darknet_openface2
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/combi_darknet_openface2
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(combi_darknet_openface2_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(combi_darknet_openface2_generate_messages combi_darknet_openface2_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg" NAME_WE)
add_dependencies(combi_darknet_openface2_generate_messages_nodejs _combi_darknet_openface2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/Faces.msg" NAME_WE)
add_dependencies(combi_darknet_openface2_generate_messages_nodejs _combi_darknet_openface2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/ActionUnit.msg" NAME_WE)
add_dependencies(combi_darknet_openface2_generate_messages_nodejs _combi_darknet_openface2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(combi_darknet_openface2_gennodejs)
add_dependencies(combi_darknet_openface2_gennodejs combi_darknet_openface2_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS combi_darknet_openface2_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(combi_darknet_openface2
  "/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/combi_darknet_openface2
)
_generate_msg_py(combi_darknet_openface2
  "/root/catkin_ws/src/combi_darknet_openface2/msg/Faces.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/combi_darknet_openface2
)
_generate_msg_py(combi_darknet_openface2
  "/root/catkin_ws/src/combi_darknet_openface2/msg/ActionUnit.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/combi_darknet_openface2
)

### Generating Services

### Generating Module File
_generate_module_py(combi_darknet_openface2
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/combi_darknet_openface2
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(combi_darknet_openface2_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(combi_darknet_openface2_generate_messages combi_darknet_openface2_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/Face.msg" NAME_WE)
add_dependencies(combi_darknet_openface2_generate_messages_py _combi_darknet_openface2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/Faces.msg" NAME_WE)
add_dependencies(combi_darknet_openface2_generate_messages_py _combi_darknet_openface2_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/combi_darknet_openface2/msg/ActionUnit.msg" NAME_WE)
add_dependencies(combi_darknet_openface2_generate_messages_py _combi_darknet_openface2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(combi_darknet_openface2_genpy)
add_dependencies(combi_darknet_openface2_genpy combi_darknet_openface2_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS combi_darknet_openface2_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/combi_darknet_openface2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/combi_darknet_openface2
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(combi_darknet_openface2_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(combi_darknet_openface2_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/combi_darknet_openface2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/combi_darknet_openface2
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(combi_darknet_openface2_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(combi_darknet_openface2_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/combi_darknet_openface2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/combi_darknet_openface2
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(combi_darknet_openface2_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(combi_darknet_openface2_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/combi_darknet_openface2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/combi_darknet_openface2
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(combi_darknet_openface2_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(combi_darknet_openface2_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/combi_darknet_openface2)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/combi_darknet_openface2\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/combi_darknet_openface2
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(combi_darknet_openface2_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(combi_darknet_openface2_generate_messages_py geometry_msgs_generate_messages_py)
endif()
