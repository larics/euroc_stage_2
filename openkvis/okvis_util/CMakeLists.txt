cmake_minimum_required(VERSION 2.8.3)
project(okvis_util)

find_package(catkin_simple REQUIRED)
catkin_simple()

if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_definitions(-fPIC -msse2 -mssse3 -march=nocona -Wextra -Winit-self 
                -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo
                -Wno-long-long)
                  
IF(NOT ${CMAKE_SYSTEM} MATCHES "Darwin-*")
  add_definitions(-std=c++0x)
ENDIF(${CMAKE_SYSTEM} MATCHES "Darwin-*")

cs_install()
cs_export()
