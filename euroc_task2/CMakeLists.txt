cmake_minimum_required(VERSION 2.8.3)
project(euroc_task2)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
 src/task2_server.cpp
 src/task2_eval.cpp
 )
target_link_libraries(${PROJECT_NAME})

###########
# BINARIES #
############
cs_add_executable(task2_server_node
  src/task2_server_node.cpp
)
cs_add_executable(task2_eval_node
  src/task2_eval_node.cpp
)

target_link_libraries(task2_server_node ${PROJECT_NAME})
target_link_libraries(task2_eval_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
