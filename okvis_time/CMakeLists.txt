cmake_minimum_required(VERSION 2.8.3)
project(okvis_time)

find_package(catkin_simple REQUIRED)
catkin_simple()

# Helen's block of happy fun make this stuff run.
if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_definitions(-O3)
add_definitions(-w)
add_definitions(-std=c++11)

cs_add_library(${PROJECT_NAME}
  src/Time.cpp
  src/Duration.cpp)

target_link_libraries(${PROJECT_NAME})

cs_install()

cs_export()
