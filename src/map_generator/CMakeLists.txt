cmake_minimum_required(VERSION 3.0.2)
project(map_generator)

message(STATUS "This is a tool for generating random gird map")
message(STATUS "Author: Zhang Zhimeng")

#set(CMAKE_VERBOSE_MAKEFILE "true")

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

include(cmake/FindEigen.cmake)
include_directories(${EIGEN_INCLUDE_DIR})

find_package(PCL REQUIRED)

find_package(
    catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    tf
)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES map_generator
    #  CATKIN_DEPENDS nav_mags roscpp tf
    #  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_executable(map_generator_3d src/map_generator_3d.cpp src/random_geometry_generator_3d.cpp)
target_link_libraries(map_generator_3d ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(map_generator_2d src/map_generator_2d.cpp src/random_geometry_generator_2d.cpp)
target_link_libraries(map_generator_2d ${catkin_LIBRARIES} ${PCL_LIBRARIES})