cmake_minimum_required(VERSION 2.8.3)
project(okvis_matcher)

find_package(catkin_simple REQUIRED)
catkin_simple()

add_definitions(-std=c++11)

# Helen's block of happy fun make this stuff run.
if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_definitions(-march=native)
add_definitions(-w)

cs_add_library(${PROJECT_NAME}
  src/DenseMatcher.cpp
  src/MatchingAlgorithm.cpp
  src/ThreadPool.cpp
  include/okvis/DenseMatcher.hpp
  include/okvis/MatchingAlgorithm.hpp
  include/okvis/ThreadPool.hpp
)

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

catkin_add_gtest(${PROJECT_NAME}_test
  test/test_main.cpp
  test/testMatcher.cpp
)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})

cs_install()
cs_export()
