cmake_minimum_required(VERSION 2.8.3)
project(mav_planning_utils)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs std_msgs visualization_msgs planning_msgs msf_timing mav_viz nlopt mav_msgs cmake_modules glog_catkin eigen_checks eigen_conversions yaml_cpp_catkin)

find_package(Eigen REQUIRED)

include_directories(include ${Eigen_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS eigen
    CATKIN_DEPENDS roscpp geometry_msgs std_msgs visualization_msgs planning_msgs msf_timing mav_viz nlopt mav_msgs glog_catkin eigen_checks
    INCLUDE_DIRS  include ${Eigen_INCLUDE_DIRS}
    LIBRARIES yaml-cpp ${PROJECT_NAME} ${PROJECT_NAME}_timing ${PROJECT_NAME}_rpoly
    CFG_EXTRAS export_flags.cmake
)

add_subdirectory(src/library)
add_subdirectory(src/test)
add_subdirectory(src/nodes)
