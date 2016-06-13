cmake_minimum_required(VERSION 2.8.3)
project(ground_truth_octree)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
 src/ground_truth_octree.cpp
 )

###########
# BINARIES #
############
cs_add_executable(ground_truth_octree_node
  src/ground_truth_octree_node.cpp
)

target_link_libraries(ground_truth_octree_node ${PROJECT_NAME} ${OCTOMAP_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()

