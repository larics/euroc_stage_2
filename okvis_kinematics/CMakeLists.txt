cmake_minimum_required(VERSION 2.8.3)
project(okvis_kinematics)

find_package(catkin_simple REQUIRED)

catkin_simple()

if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_definitions(-fPIC -march=native -Wextra -Winit-self -Woverloaded-virtual 
                -Wnon-virtual-dtor -Wsign-promo -Wno-long-long -std=c++11)
                
cs_add_library(${PROJECT_NAME} 
#  include/okvis/kinematics/operators.hpp
#  include/okvis/kinematics/Transformation.hpp
   test/TestTransformation.cpp
)
target_link_libraries(${PROJECT_NAME})

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

catkin_add_gtest(${PROJECT_NAME}_test 
  test/runTests.cpp
  test/TestTransformation.cpp
)

#cs_add_executable(pinholeCamera test/TestMultiFrame.cpp)
#target_link_libraries( pinholeCamera ${PROJECT_NAME} -lpthread)

#SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -lpthread")
#target_link_libraries( ${PROJECT_NAME}_test ${PROJECT_NAME} -lpthread)
target_link_libraries( ${PROJECT_NAME}_test -lpthread)

cs_install()
cs_export()
