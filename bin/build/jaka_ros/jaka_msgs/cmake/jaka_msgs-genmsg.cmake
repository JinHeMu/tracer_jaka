# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "jaka_msgs: 1 messages, 12 services")

set(MSG_I_FLAGS "-Ijaka_msgs:/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(jaka_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/msg/RobotMsg.msg" NAME_WE)
add_custom_target(_jaka_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_msgs" "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/msg/RobotMsg.msg" ""
)

get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/Move.srv" NAME_WE)
add_custom_target(_jaka_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_msgs" "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/Move.srv" ""
)

get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMoveEnable.srv" NAME_WE)
add_custom_target(_jaka_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_msgs" "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMoveEnable.srv" ""
)

get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMove.srv" NAME_WE)
add_custom_target(_jaka_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_msgs" "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMove.srv" ""
)

get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetTcpFrame.srv" NAME_WE)
add_custom_target(_jaka_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_msgs" "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetTcpFrame.srv" ""
)

get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetUserFrame.srv" NAME_WE)
add_custom_target(_jaka_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_msgs" "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetUserFrame.srv" ""
)

get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetPayload.srv" NAME_WE)
add_custom_target(_jaka_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_msgs" "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetPayload.srv" ""
)

get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetCollision.srv" NAME_WE)
add_custom_target(_jaka_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_msgs" "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetCollision.srv" ""
)

get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetIO.srv" NAME_WE)
add_custom_target(_jaka_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_msgs" "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetIO.srv" ""
)

get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIO.srv" NAME_WE)
add_custom_target(_jaka_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_msgs" "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIO.srv" ""
)

get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ClearError.srv" NAME_WE)
add_custom_target(_jaka_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_msgs" "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ClearError.srv" ""
)

get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetFK.srv" NAME_WE)
add_custom_target(_jaka_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_msgs" "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetFK.srv" ""
)

get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIK.srv" NAME_WE)
add_custom_target(_jaka_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaka_msgs" "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIK.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/msg/RobotMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_msgs
)

### Generating Services
_generate_srv_cpp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_cpp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMoveEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_cpp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMove.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_cpp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetTcpFrame.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_cpp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetUserFrame.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_cpp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetPayload.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_cpp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetCollision.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_cpp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_cpp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_cpp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ClearError.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_cpp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetFK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_cpp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_msgs
)

### Generating Module File
_generate_module_cpp(jaka_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(jaka_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(jaka_msgs_generate_messages jaka_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/msg/RobotMsg.msg" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_cpp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/Move.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_cpp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMoveEnable.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_cpp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMove.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_cpp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetTcpFrame.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_cpp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetUserFrame.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_cpp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetPayload.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_cpp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetCollision.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_cpp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetIO.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_cpp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIO.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_cpp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ClearError.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_cpp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetFK.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_cpp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIK.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_cpp _jaka_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jaka_msgs_gencpp)
add_dependencies(jaka_msgs_gencpp jaka_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jaka_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/msg/RobotMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_msgs
)

### Generating Services
_generate_srv_eus(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_msgs
)
_generate_srv_eus(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMoveEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_msgs
)
_generate_srv_eus(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMove.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_msgs
)
_generate_srv_eus(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetTcpFrame.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_msgs
)
_generate_srv_eus(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetUserFrame.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_msgs
)
_generate_srv_eus(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetPayload.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_msgs
)
_generate_srv_eus(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetCollision.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_msgs
)
_generate_srv_eus(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_msgs
)
_generate_srv_eus(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_msgs
)
_generate_srv_eus(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ClearError.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_msgs
)
_generate_srv_eus(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetFK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_msgs
)
_generate_srv_eus(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_msgs
)

### Generating Module File
_generate_module_eus(jaka_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(jaka_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(jaka_msgs_generate_messages jaka_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/msg/RobotMsg.msg" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_eus _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/Move.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_eus _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMoveEnable.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_eus _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMove.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_eus _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetTcpFrame.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_eus _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetUserFrame.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_eus _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetPayload.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_eus _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetCollision.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_eus _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetIO.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_eus _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIO.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_eus _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ClearError.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_eus _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetFK.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_eus _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIK.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_eus _jaka_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jaka_msgs_geneus)
add_dependencies(jaka_msgs_geneus jaka_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jaka_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/msg/RobotMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_msgs
)

### Generating Services
_generate_srv_lisp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_lisp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMoveEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_lisp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMove.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_lisp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetTcpFrame.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_lisp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetUserFrame.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_lisp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetPayload.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_lisp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetCollision.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_lisp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_lisp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_lisp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ClearError.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_lisp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetFK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_msgs
)
_generate_srv_lisp(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_msgs
)

### Generating Module File
_generate_module_lisp(jaka_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(jaka_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(jaka_msgs_generate_messages jaka_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/msg/RobotMsg.msg" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_lisp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/Move.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_lisp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMoveEnable.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_lisp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMove.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_lisp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetTcpFrame.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_lisp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetUserFrame.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_lisp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetPayload.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_lisp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetCollision.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_lisp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetIO.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_lisp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIO.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_lisp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ClearError.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_lisp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetFK.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_lisp _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIK.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_lisp _jaka_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jaka_msgs_genlisp)
add_dependencies(jaka_msgs_genlisp jaka_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jaka_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/msg/RobotMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_msgs
)

### Generating Services
_generate_srv_nodejs(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_msgs
)
_generate_srv_nodejs(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMoveEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_msgs
)
_generate_srv_nodejs(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMove.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_msgs
)
_generate_srv_nodejs(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetTcpFrame.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_msgs
)
_generate_srv_nodejs(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetUserFrame.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_msgs
)
_generate_srv_nodejs(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetPayload.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_msgs
)
_generate_srv_nodejs(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetCollision.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_msgs
)
_generate_srv_nodejs(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_msgs
)
_generate_srv_nodejs(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_msgs
)
_generate_srv_nodejs(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ClearError.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_msgs
)
_generate_srv_nodejs(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetFK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_msgs
)
_generate_srv_nodejs(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_msgs
)

### Generating Module File
_generate_module_nodejs(jaka_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(jaka_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(jaka_msgs_generate_messages jaka_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/msg/RobotMsg.msg" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_nodejs _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/Move.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_nodejs _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMoveEnable.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_nodejs _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMove.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_nodejs _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetTcpFrame.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_nodejs _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetUserFrame.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_nodejs _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetPayload.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_nodejs _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetCollision.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_nodejs _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetIO.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_nodejs _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIO.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_nodejs _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ClearError.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_nodejs _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetFK.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_nodejs _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIK.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_nodejs _jaka_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jaka_msgs_gennodejs)
add_dependencies(jaka_msgs_gennodejs jaka_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jaka_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/msg/RobotMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs
)

### Generating Services
_generate_srv_py(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/Move.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs
)
_generate_srv_py(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMoveEnable.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs
)
_generate_srv_py(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMove.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs
)
_generate_srv_py(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetTcpFrame.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs
)
_generate_srv_py(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetUserFrame.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs
)
_generate_srv_py(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetPayload.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs
)
_generate_srv_py(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetCollision.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs
)
_generate_srv_py(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs
)
_generate_srv_py(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIO.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs
)
_generate_srv_py(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ClearError.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs
)
_generate_srv_py(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetFK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs
)
_generate_srv_py(jaka_msgs
  "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIK.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs
)

### Generating Module File
_generate_module_py(jaka_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(jaka_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(jaka_msgs_generate_messages jaka_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/msg/RobotMsg.msg" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_py _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/Move.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_py _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMoveEnable.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_py _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMove.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_py _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetTcpFrame.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_py _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetUserFrame.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_py _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetPayload.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_py _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetCollision.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_py _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetIO.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_py _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIO.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_py _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ClearError.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_py _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetFK.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_py _jaka_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIK.srv" NAME_WE)
add_dependencies(jaka_msgs_generate_messages_py _jaka_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jaka_msgs_genpy)
add_dependencies(jaka_msgs_genpy jaka_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jaka_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaka_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(jaka_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/jaka_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(jaka_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaka_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(jaka_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/jaka_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(jaka_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaka_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(jaka_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
