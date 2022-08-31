cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(map)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(map src/map.cpp)
target_link_libraries(map ${PCL_LIBRARIES})
