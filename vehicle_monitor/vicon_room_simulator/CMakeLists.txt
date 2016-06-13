cmake_minimum_required(VERSION 2.8.3)
project(vicon_room_simulator)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############

###########
# BINARIES #
############
cs_add_executable(vicon_room_simulator
  src/vicon_room_simulator.cpp
)


##########
# EXPORT #
##########
cs_install()
cs_export()

