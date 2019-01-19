# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "shoebot_sew_planning: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(shoebot_sew_planning_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/calum/MotoWorkspace/src/shoebot_sew_planning/srv/SewPlanner.srv" NAME_WE)
add_custom_target(_shoebot_sew_planning_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "shoebot_sew_planning" "/home/calum/MotoWorkspace/src/shoebot_sew_planning/srv/SewPlanner.srv" "geometry_msgs/Point:geometry_msgs/Pose:std_msgs/Header:std_msgs/MultiArrayLayout:geometry_msgs/Quaternion:geometry_msgs/PoseArray:std_msgs/MultiArrayDimension:std_msgs/Float64MultiArray"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(shoebot_sew_planning
  "/home/calum/MotoWorkspace/src/shoebot_sew_planning/srv/SewPlanner.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/shoebot_sew_planning
)

### Generating Module File
_generate_module_cpp(shoebot_sew_planning
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/shoebot_sew_planning
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(shoebot_sew_planning_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(shoebot_sew_planning_generate_messages shoebot_sew_planning_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/calum/MotoWorkspace/src/shoebot_sew_planning/srv/SewPlanner.srv" NAME_WE)
add_dependencies(shoebot_sew_planning_generate_messages_cpp _shoebot_sew_planning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(shoebot_sew_planning_gencpp)
add_dependencies(shoebot_sew_planning_gencpp shoebot_sew_planning_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS shoebot_sew_planning_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(shoebot_sew_planning
  "/home/calum/MotoWorkspace/src/shoebot_sew_planning/srv/SewPlanner.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/shoebot_sew_planning
)

### Generating Module File
_generate_module_eus(shoebot_sew_planning
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/shoebot_sew_planning
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(shoebot_sew_planning_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(shoebot_sew_planning_generate_messages shoebot_sew_planning_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/calum/MotoWorkspace/src/shoebot_sew_planning/srv/SewPlanner.srv" NAME_WE)
add_dependencies(shoebot_sew_planning_generate_messages_eus _shoebot_sew_planning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(shoebot_sew_planning_geneus)
add_dependencies(shoebot_sew_planning_geneus shoebot_sew_planning_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS shoebot_sew_planning_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(shoebot_sew_planning
  "/home/calum/MotoWorkspace/src/shoebot_sew_planning/srv/SewPlanner.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/shoebot_sew_planning
)

### Generating Module File
_generate_module_lisp(shoebot_sew_planning
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/shoebot_sew_planning
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(shoebot_sew_planning_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(shoebot_sew_planning_generate_messages shoebot_sew_planning_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/calum/MotoWorkspace/src/shoebot_sew_planning/srv/SewPlanner.srv" NAME_WE)
add_dependencies(shoebot_sew_planning_generate_messages_lisp _shoebot_sew_planning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(shoebot_sew_planning_genlisp)
add_dependencies(shoebot_sew_planning_genlisp shoebot_sew_planning_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS shoebot_sew_planning_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(shoebot_sew_planning
  "/home/calum/MotoWorkspace/src/shoebot_sew_planning/srv/SewPlanner.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/shoebot_sew_planning
)

### Generating Module File
_generate_module_nodejs(shoebot_sew_planning
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/shoebot_sew_planning
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(shoebot_sew_planning_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(shoebot_sew_planning_generate_messages shoebot_sew_planning_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/calum/MotoWorkspace/src/shoebot_sew_planning/srv/SewPlanner.srv" NAME_WE)
add_dependencies(shoebot_sew_planning_generate_messages_nodejs _shoebot_sew_planning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(shoebot_sew_planning_gennodejs)
add_dependencies(shoebot_sew_planning_gennodejs shoebot_sew_planning_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS shoebot_sew_planning_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(shoebot_sew_planning
  "/home/calum/MotoWorkspace/src/shoebot_sew_planning/srv/SewPlanner.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/shoebot_sew_planning
)

### Generating Module File
_generate_module_py(shoebot_sew_planning
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/shoebot_sew_planning
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(shoebot_sew_planning_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(shoebot_sew_planning_generate_messages shoebot_sew_planning_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/calum/MotoWorkspace/src/shoebot_sew_planning/srv/SewPlanner.srv" NAME_WE)
add_dependencies(shoebot_sew_planning_generate_messages_py _shoebot_sew_planning_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(shoebot_sew_planning_genpy)
add_dependencies(shoebot_sew_planning_genpy shoebot_sew_planning_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS shoebot_sew_planning_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/shoebot_sew_planning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/shoebot_sew_planning
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(shoebot_sew_planning_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(shoebot_sew_planning_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/shoebot_sew_planning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/shoebot_sew_planning
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(shoebot_sew_planning_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(shoebot_sew_planning_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/shoebot_sew_planning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/shoebot_sew_planning
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(shoebot_sew_planning_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(shoebot_sew_planning_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/shoebot_sew_planning)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/shoebot_sew_planning
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(shoebot_sew_planning_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(shoebot_sew_planning_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/shoebot_sew_planning)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/shoebot_sew_planning\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/shoebot_sew_planning
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(shoebot_sew_planning_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(shoebot_sew_planning_generate_messages_py geometry_msgs_generate_messages_py)
endif()
