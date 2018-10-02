# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "test_headpose_estimate: 3 messages, 0 services")

set(MSG_I_FLAGS "-Itest_headpose_estimate:/root/catkin_ws/src/test_headpose_estimate/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(test_headpose_estimate_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg" NAME_WE)
add_custom_target(_test_headpose_estimate_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "test_headpose_estimate" "/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg" "geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Vector3"
)

get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/ActionUnit.msg" NAME_WE)
add_custom_target(_test_headpose_estimate_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "test_headpose_estimate" "/root/catkin_ws/src/test_headpose_estimate/msg/ActionUnit.msg" ""
)

get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/Faces.msg" NAME_WE)
add_custom_target(_test_headpose_estimate_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "test_headpose_estimate" "/root/catkin_ws/src/test_headpose_estimate/msg/Faces.msg" "std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/Vector3:geometry_msgs/Pose:test_headpose_estimate/Face"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(test_headpose_estimate
  "/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/test_headpose_estimate
)
_generate_msg_cpp(test_headpose_estimate
  "/root/catkin_ws/src/test_headpose_estimate/msg/ActionUnit.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/test_headpose_estimate
)
_generate_msg_cpp(test_headpose_estimate
  "/root/catkin_ws/src/test_headpose_estimate/msg/Faces.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/test_headpose_estimate
)

### Generating Services

### Generating Module File
_generate_module_cpp(test_headpose_estimate
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/test_headpose_estimate
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(test_headpose_estimate_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(test_headpose_estimate_generate_messages test_headpose_estimate_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg" NAME_WE)
add_dependencies(test_headpose_estimate_generate_messages_cpp _test_headpose_estimate_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/ActionUnit.msg" NAME_WE)
add_dependencies(test_headpose_estimate_generate_messages_cpp _test_headpose_estimate_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/Faces.msg" NAME_WE)
add_dependencies(test_headpose_estimate_generate_messages_cpp _test_headpose_estimate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(test_headpose_estimate_gencpp)
add_dependencies(test_headpose_estimate_gencpp test_headpose_estimate_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS test_headpose_estimate_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(test_headpose_estimate
  "/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/test_headpose_estimate
)
_generate_msg_eus(test_headpose_estimate
  "/root/catkin_ws/src/test_headpose_estimate/msg/ActionUnit.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/test_headpose_estimate
)
_generate_msg_eus(test_headpose_estimate
  "/root/catkin_ws/src/test_headpose_estimate/msg/Faces.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/test_headpose_estimate
)

### Generating Services

### Generating Module File
_generate_module_eus(test_headpose_estimate
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/test_headpose_estimate
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(test_headpose_estimate_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(test_headpose_estimate_generate_messages test_headpose_estimate_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg" NAME_WE)
add_dependencies(test_headpose_estimate_generate_messages_eus _test_headpose_estimate_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/ActionUnit.msg" NAME_WE)
add_dependencies(test_headpose_estimate_generate_messages_eus _test_headpose_estimate_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/Faces.msg" NAME_WE)
add_dependencies(test_headpose_estimate_generate_messages_eus _test_headpose_estimate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(test_headpose_estimate_geneus)
add_dependencies(test_headpose_estimate_geneus test_headpose_estimate_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS test_headpose_estimate_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(test_headpose_estimate
  "/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/test_headpose_estimate
)
_generate_msg_lisp(test_headpose_estimate
  "/root/catkin_ws/src/test_headpose_estimate/msg/ActionUnit.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/test_headpose_estimate
)
_generate_msg_lisp(test_headpose_estimate
  "/root/catkin_ws/src/test_headpose_estimate/msg/Faces.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/test_headpose_estimate
)

### Generating Services

### Generating Module File
_generate_module_lisp(test_headpose_estimate
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/test_headpose_estimate
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(test_headpose_estimate_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(test_headpose_estimate_generate_messages test_headpose_estimate_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg" NAME_WE)
add_dependencies(test_headpose_estimate_generate_messages_lisp _test_headpose_estimate_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/ActionUnit.msg" NAME_WE)
add_dependencies(test_headpose_estimate_generate_messages_lisp _test_headpose_estimate_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/Faces.msg" NAME_WE)
add_dependencies(test_headpose_estimate_generate_messages_lisp _test_headpose_estimate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(test_headpose_estimate_genlisp)
add_dependencies(test_headpose_estimate_genlisp test_headpose_estimate_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS test_headpose_estimate_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(test_headpose_estimate
  "/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/test_headpose_estimate
)
_generate_msg_nodejs(test_headpose_estimate
  "/root/catkin_ws/src/test_headpose_estimate/msg/ActionUnit.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/test_headpose_estimate
)
_generate_msg_nodejs(test_headpose_estimate
  "/root/catkin_ws/src/test_headpose_estimate/msg/Faces.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/test_headpose_estimate
)

### Generating Services

### Generating Module File
_generate_module_nodejs(test_headpose_estimate
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/test_headpose_estimate
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(test_headpose_estimate_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(test_headpose_estimate_generate_messages test_headpose_estimate_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg" NAME_WE)
add_dependencies(test_headpose_estimate_generate_messages_nodejs _test_headpose_estimate_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/ActionUnit.msg" NAME_WE)
add_dependencies(test_headpose_estimate_generate_messages_nodejs _test_headpose_estimate_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/Faces.msg" NAME_WE)
add_dependencies(test_headpose_estimate_generate_messages_nodejs _test_headpose_estimate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(test_headpose_estimate_gennodejs)
add_dependencies(test_headpose_estimate_gennodejs test_headpose_estimate_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS test_headpose_estimate_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(test_headpose_estimate
  "/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/test_headpose_estimate
)
_generate_msg_py(test_headpose_estimate
  "/root/catkin_ws/src/test_headpose_estimate/msg/ActionUnit.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/test_headpose_estimate
)
_generate_msg_py(test_headpose_estimate
  "/root/catkin_ws/src/test_headpose_estimate/msg/Faces.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/test_headpose_estimate
)

### Generating Services

### Generating Module File
_generate_module_py(test_headpose_estimate
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/test_headpose_estimate
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(test_headpose_estimate_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(test_headpose_estimate_generate_messages test_headpose_estimate_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/Face.msg" NAME_WE)
add_dependencies(test_headpose_estimate_generate_messages_py _test_headpose_estimate_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/ActionUnit.msg" NAME_WE)
add_dependencies(test_headpose_estimate_generate_messages_py _test_headpose_estimate_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/root/catkin_ws/src/test_headpose_estimate/msg/Faces.msg" NAME_WE)
add_dependencies(test_headpose_estimate_generate_messages_py _test_headpose_estimate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(test_headpose_estimate_genpy)
add_dependencies(test_headpose_estimate_genpy test_headpose_estimate_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS test_headpose_estimate_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/test_headpose_estimate)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/test_headpose_estimate
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(test_headpose_estimate_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(test_headpose_estimate_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/test_headpose_estimate)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/test_headpose_estimate
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(test_headpose_estimate_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(test_headpose_estimate_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/test_headpose_estimate)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/test_headpose_estimate
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(test_headpose_estimate_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(test_headpose_estimate_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/test_headpose_estimate)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/test_headpose_estimate
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(test_headpose_estimate_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(test_headpose_estimate_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/test_headpose_estimate)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/test_headpose_estimate\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/test_headpose_estimate
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(test_headpose_estimate_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(test_headpose_estimate_generate_messages_py geometry_msgs_generate_messages_py)
endif()
