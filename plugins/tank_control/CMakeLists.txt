cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(tank_control_plugin SHARED tank_control.cc)
set_property(TARGET tank_control_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(tank_control_plugin ${GAZEBO_libraries})
