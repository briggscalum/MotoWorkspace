# Install script for directory: /home/calum/MotoWorkspace/src/motoman/motoman_sia10f_support

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/calum/MotoWorkspace/build/motoman/motoman_sia10f_support/catkin_generated/installspace/motoman_sia10f_support.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/motoman_sia10f_support/cmake" TYPE FILE FILES
    "/home/calum/MotoWorkspace/build/motoman/motoman_sia10f_support/catkin_generated/installspace/motoman_sia10f_supportConfig.cmake"
    "/home/calum/MotoWorkspace/build/motoman/motoman_sia10f_support/catkin_generated/installspace/motoman_sia10f_supportConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/motoman_sia10f_support" TYPE FILE FILES "/home/calum/MotoWorkspace/src/motoman/motoman_sia10f_support/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/motoman_sia10f_support" TYPE DIRECTORY FILES
    "/home/calum/MotoWorkspace/src/motoman/motoman_sia10f_support/config"
    "/home/calum/MotoWorkspace/src/motoman/motoman_sia10f_support/launch"
    "/home/calum/MotoWorkspace/src/motoman/motoman_sia10f_support/meshes"
    "/home/calum/MotoWorkspace/src/motoman/motoman_sia10f_support/urdf"
    )
endif()

