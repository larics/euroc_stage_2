cmake_minimum_required(VERSION 2.8.3)
project(okvis_ceres)

find_package(catkin_simple REQUIRED)

catkin_simple()

add_definitions(-std=c++11)

if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_definitions(-fPIC -march=native -Wextra -Winit-self -Woverloaded-virtual 
                -Wnon-virtual-dtor -Wsign-promo -Wno-long-long )
                
cs_add_library(${PROJECT_NAME} 
  src/PoseParameterBlock.cpp
  src/SpeedAndBiasParameterBlock.cpp
  src/HomogeneousPointParameterBlock.cpp
  src/HomogeneousPointLocalParameterization.cpp
  src/PoseLocalParameterization.cpp
  src/ImuError.cpp
  src/PoseError.cpp
  src/RelativePoseError.cpp
  src/SpeedAndBiasError.cpp
  src/IdProvider.cpp
  src/Map.cpp
  src/MarginalizationError.cpp
  src/HomogeneousPointError.cpp
  src/Estimator.cpp
  src/LocalParamizationAdditionalInterfaces.cpp
  include/okvis/Estimator.hpp
  include/okvis/ceres/CeresIterationCallback.hpp
)
target_link_libraries(${PROJECT_NAME})

# Avoid clash with tr1::tuple: https://code.google.com/p/googletest/source/browse/trunk/README?r=589#257
add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=0)

catkin_add_gtest(${PROJECT_NAME}_test 
  test/test_main.cpp
  test/TestEstimator.cpp
  test/TestHomogeneousPointError.cpp
  test/TestReprojectionError.cpp
  test/TestImuError.cpp
  test/TestMap.cpp
  test/TestMarginalization.cpp
)

#cs_add_executable(estimator test/TestMarginalization.cpp)
#target_link_libraries( estimator ${PROJECT_NAME} )

target_link_libraries( ${PROJECT_NAME}_test ${PROJECT_NAME}  -lpthread)

cs_install()
cs_export()
