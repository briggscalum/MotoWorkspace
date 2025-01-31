# Install script for directory: /home/calum/MotoWorkspace/src/robotiq/robotiq_3f_gripper_articulated_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/calum/MotoWorkspace/install")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_3f_gripper_articulated_msgs/msg" TYPE FILE FILES
    "/home/calum/MotoWorkspace/src/robotiq/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotInput.msg"
    "/home/calum/MotoWorkspace/src/robotiq/robotiq_3f_gripper_articulated_msgs/msg/Robotiq3FGripperRobotOutput.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_3f_gripper_articulated_msgs/cmake" TYPE FILE FILES "/home/calum/MotoWorkspace/build/robotiq/robotiq_3f_gripper_articulated_msgs/catkin_generated/installspace/robotiq_3f_gripper_articulated_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/calum/MotoWorkspace/devel/include/robotiq_3f_gripper_articulated_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/calum/MotoWorkspace/devel/share/roseus/ros/robotiq_3f_gripper_articulated_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/calum/MotoWorkspace/devel/share/common-lisp/ros/robotiq_3f_gripper_articulated_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/calum/MotoWorkspace/devel/share/gennodejs/ros/robotiq_3f_gripper_articulated_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/calum/MotoWorkspace/devel/lib/python2.7/dist-packages/robotiq_3f_gripper_articulated_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/calum/MotoWorkspace/devel/lib/python2.7/dist-packages/robotiq_3f_gripper_articulated_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/calum/MotoWorkspace/build/robotiq/robotiq_3f_gripper_articulated_msgs/catkin_generated/installspace/robotiq_3f_gripper_articulated_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_3f_gripper_articulated_msgs/cmake" TYPE FILE FILES "/home/calum/MotoWorkspace/build/robotiq/robotiq_3f_gripper_articulated_msgs/catkin_generated/installspace/robotiq_3f_gripper_articulated_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_3f_gripper_articulated_msgs/cmake" TYPE FILE FILES
    "/home/calum/MotoWorkspace/build/robotiq/robotiq_3f_gripper_articulated_msgs/catkin_generated/installspace/robotiq_3f_gripper_articulated_msgsConfig.cmake"
    "/home/calum/MotoWorkspace/build/robotiq/robotiq_3f_gripper_articulated_msgs/catkin_generated/installspace/robotiq_3f_gripper_articulated_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_3f_gripper_articulated_msgs" TYPE FILE FILES "/home/calum/MotoWorkspace/src/robotiq/robotiq_3f_gripper_articulated_msgs/package.xml")
endif()

