cmake_minimum_required(VERSION 2.8.3)
project(euroc_task5)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
 src/task5_server.cpp
 )

############
# BINARIES #
############
cs_add_executable(task5_server_node
  src/task5_server_node.cpp
)

target_link_libraries(task5_server_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
