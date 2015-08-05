cmake_minimum_required(VERSION 2.8.3)
project(okvis_node)

find_package(catkin_simple REQUIRED)

catkin_simple()

IF(visensor_msgs_FOUND)
  add_definitions(-DHAVE_VISENSOR)
  include_directories(${visensor_msgs_PACKAGE_PATH}/include)
  link_directories(${visensor_msgs_PACKAGE_PATH}/lib)
  MESSAGE(STATUS "Found visensor messages package. Setting HAVE_VISENSOR flag.")
ELSEIF(visensor_node_FOUND)
  add_definitions(-DHAVE_VISENSOR)
  add_definitions(-DUSE_VISENSORNODE_V1_1)
  MESSAGE(STATUS "Did not find visensor_msgs package but found visensor node.")
  MESSAGE(STATUS "Setting HAVE_VISENSOR and USE_VISENSORNODE_V1_1")
ENDIF()

IF(libvisensor_FOUND)
  add_definitions(-DHAVE_LIBVISENSOR)
  MESSAGE(STATUS "Found libvisensor. Setting HAVE_LIBVISENSOR flag.")
ENDIF()

if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED COMPONENTS system serialization filesystem)

add_definitions(-fPIC -march=native -Wextra -Winit-self -Woverloaded-virtual 
                -Wnon-virtual-dtor -Wsign-promo -Wno-long-long -std=c++0x)

cs_add_library(okvis_node_libs src/Subscriber.cpp
                               src/Publisher.cpp
                               src/RosParametersReader.cpp
                               include/okvis/Subscriber.hpp
                               include/okvis/Publisher.hpp
                               include/okvis/RosParametersReader.hpp)

target_link_libraries(okvis_node_libs ${Boost_LIBRARIES})

cs_add_executable(okvis_node src/okvis_node.cpp)
target_link_libraries(okvis_node okvis_node_libs)

cs_add_executable(okvis_node_synchronous src/okvis_node_synchronous.cpp)
target_link_libraries(okvis_node_synchronous okvis_node_libs)

#cs_add_executable(okvis_node_backend src/okvis_node_backend.cpp)
#target_link_libraries(okvis_node_backend okvis_node_libs)

cs_install()
cs_export()
