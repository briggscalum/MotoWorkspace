# Install script for directory: /home/calum/MotoWorkspace/src/robotiq/robotiq_2f_gripper_control

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_gripper_control/msg" TYPE FILE FILES
    "/home/calum/MotoWorkspace/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_input.msg"
    "/home/calum/MotoWorkspace/src/robotiq/robotiq_2f_gripper_control/msg/Robotiq2FGripper_robot_output.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  include("/home/calum/MotoWorkspace/build/robotiq/robotiq_2f_gripper_control/catkin_generated/safe_execute_install.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_gripper_control/cmake" TYPE FILE FILES "/home/calum/MotoWorkspace/build/robotiq/robotiq_2f_gripper_control/catkin_generated/installspace/robotiq_2f_gripper_control-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/calum/MotoWorkspace/devel/include/robotiq_2f_gripper_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/calum/MotoWorkspace/devel/share/roseus/ros/robotiq_2f_gripper_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/calum/MotoWorkspace/devel/share/common-lisp/ros/robotiq_2f_gripper_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/calum/MotoWorkspace/devel/share/gennodejs/ros/robotiq_2f_gripper_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/calum/MotoWorkspace/devel/lib/python2.7/dist-packages/robotiq_2f_gripper_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/calum/MotoWorkspace/devel/lib/python2.7/dist-packages/robotiq_2f_gripper_control" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/calum/MotoWorkspace/devel/lib/python2.7/dist-packages/robotiq_2f_gripper_control" FILES_MATCHING REGEX "/home/calum/MotoWorkspace/devel/lib/python2.7/dist-packages/robotiq_2f_gripper_control/.+/__init__.pyc?$")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/calum/MotoWorkspace/build/robotiq/robotiq_2f_gripper_control/catkin_generated/installspace/robotiq_2f_gripper_control.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_gripper_control/cmake" TYPE FILE FILES "/home/calum/MotoWorkspace/build/robotiq/robotiq_2f_gripper_control/catkin_generated/installspace/robotiq_2f_gripper_control-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_gripper_control/cmake" TYPE FILE FILES
    "/home/calum/MotoWorkspace/build/robotiq/robotiq_2f_gripper_control/catkin_generated/installspace/robotiq_2f_gripper_controlConfig.cmake"
    "/home/calum/MotoWorkspace/build/robotiq/robotiq_2f_gripper_control/catkin_generated/installspace/robotiq_2f_gripper_controlConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_gripper_control" TYPE FILE FILES "/home/calum/MotoWorkspace/src/robotiq/robotiq_2f_gripper_control/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robotiq_2f_gripper_control" TYPE PROGRAM FILES
    "/home/calum/MotoWorkspace/src/robotiq/robotiq_2f_gripper_control/nodes/Robotiq2FGripperSimpleController.py"
    "/home/calum/MotoWorkspace/src/robotiq/robotiq_2f_gripper_control/nodes/Robotiq2FGripperStatusListener.py"
    "/home/calum/MotoWorkspace/src/robotiq/robotiq_2f_gripper_control/nodes/Robotiq2FGripperTcpNode.py"
    "/home/calum/MotoWorkspace/src/robotiq/robotiq_2f_gripper_control/nodes/Robotiq2FGripperRtuNode.py"
    )
endif()

