cmake_minimum_required(VERSION 2.8.3)
project(detector)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    cv_bridge
    message_filters
)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

find_package(OpenCV REQUIRED)
#include(/home/nv/opencv-3.4.10/build/OpenCVConfig.cmake)
#message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES detector
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS OpenCV Eigen Boost
  )

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  # Set manually because Eigen sets a non standard INCLUDE DIR
  ${EIGEN3_INCLUDE_DIR}
  # Set because Boost is an internal dependency, not transitive.
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare cpp executables
add_library(detector 
    # src/color_obj_detect_node.cpp
    src/color_detector.cpp
    src/kalman_filter.cpp)

## Add dependencies to exported targets, like ROS msgs or srvs
#add_dependencies(detector
#  ${catkin_EXPORTED_TARGETS}
#)

target_link_libraries(detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)