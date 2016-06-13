cmake_minimum_required(VERSION 2.8.3)
project(euroc_task3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
 src/task3_eval.cpp
 src/task3_server.cpp
 )

############
# BINARIES #
############
cs_add_executable(task3_server_node
  src/task3_server_node.cpp
)
cs_add_executable(task3_eval_node
  src/task3_eval_node.cpp
)

target_link_libraries(task3_server_node ${PROJECT_NAME})
target_link_libraries(task3_eval_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
