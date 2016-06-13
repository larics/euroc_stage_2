cmake_minimum_required(VERSION 2.8.3)
project(mav_viz)

find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs cmake_modules)
find_package(Eigen REQUIRED)

include_directories(include ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Release)

catkin_package(
    DEPENDS eigen
    CATKIN_DEPENDS  roscpp visualization_msgs
    INCLUDE_DIRS  include ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
    LIBRARIES mav_viz
)

add_subdirectory(src/libmav_viz)
add_subdirectory(src/nodes)

