cmake_minimum_required(VERSION 3.0)

# name of the project (change later)
project(nist_first_responder_automation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

# find the following non-ROS related dependencies
find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Boost REQUIRED)
find_package(Eigen 3.1.2 REQUIRED)

#INCLUDE_DIRECTORIES ("$ENV{EIGEN3_INCLUDE_DIR}")
include_directories(${Eigen_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp
)


find_library (GeographicLib_LIBRARIES Geographic
  PATHS "${CMAKE_INSTALL_PREFIX}/../GeographicLib/lib")

if (GeographicLib_LIBRARIES)
  get_filename_component (GeographicLib_LIBRARY_DIRS
    "${GeographicLib_LIBRARIES}" PATH)

  get_filename_component (_ROOT_DIR "${GeographicLib_LIBRARY_DIRS}" PATH)
  set (GeographicLib_INCLUDE_DIRS "${_ROOT_DIR}/../include")
  set (GeographicLib_BINARY_DIRS "${_ROOT_DIR}/../bin")

  # Required for Debian-alike systems
  find_library (GeographicLib_LIBRARIES_DEBIAN libGeographic.so
    PATHS "/usr/lib/x86_64-linux-gnu")
  if (GeographicLib_LIBRARIES_DEBIAN)
    set (GeographicLib_LIBRARIES ${GeographicLib_LIBRARIES_DEBIAN})
    get_filename_component (GeographicLib_LIBRARY_DIRS
      "${GeographicLib_LIBRARIES}" PATH)
  endif ()
endif ()

catkin_package(
  DEPENDS Boost Eigen roscpp
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs 
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

###########################################################
# EVERYTIME YOU MAKE A NEW CPP FILE, EDIT THE LINES BELOW #
###########################################################
set(NFRA_LIB_SRCS
    src/utils.cpp
)

# Create library, link external libraries against it
add_library(NFRA-lib ${NFRA_LIB_SRCS})
target_include_directories(NFRA-lib PUBLIC ${CMAKE_SOURCE_DIR}/include ${CMAKE_BINARY_DIR}/include)
target_link_libraries(NFRA-lib ${catkin_LIBRARIES})

## The recommended prefix ensures that target names across packages don't collide
## add_executable(NFRA_node src/project.cpp)
add_executable(NFRA_node_wall src/wall.cpp)
add_executable(generate_waypoints_node src/generate_waypoints.cpp)

## Specify libraries to link a library or executable target against
## target_link_libraries(NFRA_node ${catkin_LIBRARIES} NFRA-lib)
target_link_libraries(NFRA_node_wall ${catkin_LIBRARIES} NFRA-lib)
target_link_libraries(generate_waypoints_node ${catkin_LIBRARIES})