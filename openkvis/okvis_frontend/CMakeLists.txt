cmake_minimum_required(VERSION 2.8.3)
project(okvis_frontend)

find_package(catkin_simple REQUIRED)

add_definitions(-std=c++11)   

catkin_simple()

if(NOT DO_TIMING)
  add_definitions(-DDEACTIVATE_TIMERS)
  message(STATUS "Deactivating timers.")
endif()

if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_definitions(-fPIC -march=native -Wextra -Winit-self 
                -Woverloaded-virtual -Wnon-virtual-dtor -Wsign-promo
                -Wno-long-long)               

cs_add_library(${PROJECT_NAME} 
  src/Frontend.cpp
  src/VioKeyframeWindowMatchingAlgorithm.cpp
  src/stereo_triangulation.cpp
  src/ProbabilisticStereoTriangulator.cpp
  src/FrameNoncentralAbsoluteAdapter.cpp
  src/FrameRelativeAdapter.cpp
  include/okvis/Frontend.hpp
  include/okvis/VioKeyframeWindowMatchingAlgorithm.hpp
  include/okvis/triangulation/stereo_triangulation.hpp
  include/okvis/triangulation/ProbabilisticStereoTriangulator.hpp
  include/opengv/absolute_pose/FrameNoncentralAbsoluteAdapter.hpp
  include/opengv/relative_pose/FrameRelativeAdapter.hpp
  include/opengv/sac_problems/absolute_pose/FrameAbsolutePoseSacProblem.hpp
  include/opengv/sac_problems/relative_pose/FrameRelativePoseSacProblem.hpp
  include/opengv/sac_problems/relative_pose/FrameRotationOnlySacProblem.hpp
)

cs_install()
cs_export()
