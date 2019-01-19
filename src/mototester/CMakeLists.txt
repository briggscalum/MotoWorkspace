cmake_minimum_required(VERSION 2.8.3)
project(mototester)

add_compile_options(-std=c++11)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  genmsg 
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  message_generation)

## Declare a catkin package
catkin_package(INCLUDE_DIRS include)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CATKIN_DEVEL_PREFIX}/include
)

add_executable(mototester_node src/mototester_node.cpp)


target_link_libraries(mototester_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
