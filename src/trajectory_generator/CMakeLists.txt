cmake_minimum_required(VERSION 3.0.2)
project(trajectory_generator)

message(STATUS "This is a library for generating trajectory")
message(STATUS "Author: Zhang Zhimeng")

#set(CMAKE_VERBOSE_MAKEFILE "true")

# Compiler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_FLAGS} doesn't have C++11 support.
                         Please use a different C++ compiler")
endif ()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

# Eigen
include(cmake/FindEigen.cmake)
include_directories(${EIGEN_INCLUDE_DIR})

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# glog
include(cmake/glog.cmake)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(
    catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES trajectory_generator
    #  CATKIN_DEPENDS nav_msgs roscpp
    #  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(
    TRAJECTORY_GENERATOR_LIB SHARED
    src/minimum_snap/minimum_snap.cpp
    src/minimum_snap/minimum_snap_2d_flow.cpp
    src/subscriber/path_subscriber.cpp
    src/subscriber/pointcloud_subscriber.cpp
)
target_link_libraries(
    TRAJECTORY_GENERATOR_LIB
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${GLOG_LIBRARIES}
)

add_executable(run_minimum_snap_2d app/run_minimum_snap_2d.cpp)
target_link_libraries(run_minimum_snap_2d TRAJECTORY_GENERATOR_LIB)