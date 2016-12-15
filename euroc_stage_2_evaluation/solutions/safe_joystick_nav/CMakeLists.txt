cmake_minimum_required(VERSION 2.8.3)
project(safe_joystick_nav)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/safe_joystick_nav.cpp
)

############
# BINARIES #
############
cs_add_executable(safe_joystick_nav_node
  src/safe_joystick_nav_node.cpp
)
target_link_libraries(safe_joystick_nav_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
