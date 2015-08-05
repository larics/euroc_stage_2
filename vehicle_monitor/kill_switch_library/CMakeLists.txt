cmake_minimum_required(VERSION 2.8.0)
project(kill_switch_library)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(kill_switch_library
  src/library/kill_switch.cpp
  src/library/uart.cpp
)

cs_add_executable(kill_switch_node 
  src/node/kill_switch_node.cpp
)

add_subdirectory(src/test)

target_link_libraries(kill_switch_node kill_switch_library)

cs_install()
cs_export()

