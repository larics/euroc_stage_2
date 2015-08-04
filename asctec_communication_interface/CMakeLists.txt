cmake_minimum_required(VERSION 2.8.3)
project(asctec_communication_interface)
find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp)

find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

catkin_package(
DEPENDS Eigen
CATKIN_DEPENDS roscpp
INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} include
LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_uart ${PROJECT_NAME}_c
)

add_subdirectory(src/lib)
add_subdirectory(src/nodes)
add_subdirectory(src/test)