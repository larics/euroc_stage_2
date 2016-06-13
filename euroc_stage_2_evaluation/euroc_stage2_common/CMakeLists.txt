cmake_minimum_required(VERSION 2.8.3)
project(euroc_stage2_common)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
 src/results_file_writer.cpp
 src/trajectory_evaluator.cpp
 src/trajectory_interface.cpp
 src/waypoint.cpp
 src/eval_base.cpp
 src/csv_parser.cpp
 )

##########
# EXPORT #
##########
cs_install()
cs_export()
