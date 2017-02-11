cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(fpfh)

find_package(PCL 1.8 REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (fpfh fpfh.cpp)
target_link_libraries (fpfh ${PCL_LIBRARIES})
