cmake_minimum_required(VERSION 2.8.3)
project(okvis_common)

find_package(catkin_simple REQUIRED)
catkin_simple()

if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

IF(libvisensor_FOUND)
  add_definitions(-DHAVE_LIBVISENSOR)
  MESSAGE(STATUS "Found libvisensor. Setting HAVE_LIBVISENSOR flag.")
ENDIF()

add_definitions(-fPIC -msse2 -mssse3 -march=nocona -Wextra -Winit-self 
                -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo
                -Wno-long-long)
add_definitions(-std=c++11)

cs_add_library(${PROJECT_NAME} 
  src/VioInterface.cpp
  src/VioParametersReader.cpp
  include/okvis/FrameTypedefs.hpp
  include/okvis/Measurements.hpp
  include/okvis/Parameters.hpp
  include/okvis/Variables.hpp
  include/okvis/VioBackendInterface.hpp
  include/okvis/VioFrontendInterface.hpp
  include/okvis/VioInterface.hpp
  include/okvis/VioParametersReader.hpp
)

cs_install()
cs_export()
