# Install script for directory: /home/a/jaka_zu5/src/jaka_ros/jaka_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/a/jaka_zu5/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jaka_msgs/msg" TYPE FILE FILES "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/msg/RobotMsg.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jaka_msgs/srv" TYPE FILE FILES
    "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/Move.srv"
    "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMoveEnable.srv"
    "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ServoMove.srv"
    "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetTcpFrame.srv"
    "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetUserFrame.srv"
    "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetPayload.srv"
    "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetCollision.srv"
    "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/SetIO.srv"
    "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIO.srv"
    "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/ClearError.srv"
    "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetFK.srv"
    "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/srv/GetIK.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jaka_msgs/cmake" TYPE FILE FILES "/home/a/jaka_zu5/build/jaka_ros/jaka_msgs/catkin_generated/installspace/jaka_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/a/jaka_zu5/devel/include/jaka_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/a/jaka_zu5/devel/share/roseus/ros/jaka_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/a/jaka_zu5/devel/share/common-lisp/ros/jaka_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/a/jaka_zu5/devel/share/gennodejs/ros/jaka_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/a/jaka_zu5/devel/lib/python3/dist-packages/jaka_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/a/jaka_zu5/devel/lib/python3/dist-packages/jaka_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/a/jaka_zu5/build/jaka_ros/jaka_msgs/catkin_generated/installspace/jaka_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jaka_msgs/cmake" TYPE FILE FILES "/home/a/jaka_zu5/build/jaka_ros/jaka_msgs/catkin_generated/installspace/jaka_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jaka_msgs/cmake" TYPE FILE FILES
    "/home/a/jaka_zu5/build/jaka_ros/jaka_msgs/catkin_generated/installspace/jaka_msgsConfig.cmake"
    "/home/a/jaka_zu5/build/jaka_ros/jaka_msgs/catkin_generated/installspace/jaka_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/jaka_msgs" TYPE FILE FILES "/home/a/jaka_zu5/src/jaka_ros/jaka_msgs/package.xml")
endif()

