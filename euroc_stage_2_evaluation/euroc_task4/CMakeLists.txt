cmake_minimum_required(VERSION 2.8.3)
project(euroc_task4)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
# Have to do this manually?
find_package(dynamicEDT3D REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/task4_server.cpp
  src/task4_eval.cpp
)
target_link_libraries(${PROJECT_NAME} ${DYNAMICEDT3D_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(task4_server_node
  src/task4_server_node.cpp
)
target_link_libraries(task4_server_node ${PROJECT_NAME})

cs_add_executable(task4_eval_node
  src/task4_eval_node.cpp
)
target_link_libraries(task4_eval_node ${PROJECT_NAME})

cs_add_executable(task4_predict_score
  src/task4_predict_score.cpp
)
target_link_libraries(task4_predict_score ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
