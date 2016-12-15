cmake_minimum_required(VERSION 2.8.3)
project(euroc_vicon_transmission)

## Find catkin and any catkin packages
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -w)

############
# PROTOBUF #
############
set(PROTO_DIR proto/euroc_vicon_transmission)
add_subdirectory(${PROTO_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/${PROTO_DIR})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/socket.cpp
  src/server_socket.cpp
  src/client_socket.cpp
)

############
# BINARIES #
############
cs_add_executable(chatter_node
  src/chatter
)
target_link_libraries(chatter_node ${PROJECT_NAME})

cs_add_executable(vicon_pub_node
  src/vicon_pub)
target_link_libraries(vicon_pub_node PROTO_LIB ${PROJECT_NAME})

cs_add_executable(vicon_transmitter_node
src/vicon_transmitter)
target_link_libraries(vicon_transmitter_node PROTO_LIB ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
