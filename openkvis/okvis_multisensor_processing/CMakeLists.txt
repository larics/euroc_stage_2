cmake_minimum_required(VERSION 2.8.3)
project(okvis_multisensor_processing)

find_package(catkin_simple REQUIRED)

catkin_simple()

if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT DO_TIMING)
  add_definitions(-DDEACTIVATE_TIMERS)
  message(STATUS "Deactivating timers.")
endif()

cs_add_library(${PROJECT_NAME}
  src/ThreadedKFVio.cpp
  src/ImuFrameSynchronizer.cpp
  src/FrameSynchronizer.cpp
  src/VioVisualizer.cpp
  include/okvis/ThreadedKFVio.hpp
  include/okvis/ImuFrameSynchronizer.hpp
  include/okvis/FrameSynchronizer.hpp
  include/okvis/VioVisualizer.hpp
  include/okvis/threadsafe/ThreadsafeQueue.hpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_definitions(-fPIC -march=native -Wextra -Winit-self -Woverloaded-virtual 
                -Wnon-virtual-dtor -Wsign-promo -Wno-long-long -std=c++0x)
                          
################################
# GTest
################################
    set(GTEST_FROM_SOURCE_FOUND 1)
################################

catkin_add_gtest(${PROJECT_NAME}_tests
  src/ThreadedKFVio.cpp
  src/ImuFrameSynchronizer.cpp
  src/FrameSynchronizer.cpp
  src/VioVisualizer.cpp
  test/FrameSynchronizer_test.cpp
  test/ImuFrameSynchronizer_test.cpp
  test/test_main.cpp
  test/testThreading.cpp
  test/testDataFlow.cpp
  test/testSynchronizer.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
)
target_link_libraries(${PROJECT_NAME}_tests ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_tests PROPERTIES COMPILE_FLAGS -DUSE_MOCK)

cs_install()
cs_export()
