# - Config file for the soem package
# It defines the following variables
#  soem_INCLUDE_DIRS - include directories for soem
#  soem_LIBRARIES    - libraries to link against
 
# Compute paths
get_filename_component(soem_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(soem_INCLUDE_DIRS "/home/calum/MotoWorkspace/install/include/soem")
 
# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET soem AND NOT soem_BINARY_DIR)
  include("${soem_CMAKE_DIR}/SoemTargets.cmake")
endif()
 
# These are IMPORTED targets created by SoemTargets.cmake
set(soem_LIBRARIES osal oshw soem)

