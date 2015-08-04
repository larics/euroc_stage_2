cmake_minimum_required(VERSION 2.8.3)
project(aslam_install)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES
  CATKIN_DEPENDS 
  DEPENDS
)
catkin_python_setup()
#catkin_add_env_hooks(10.aslam_install SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
