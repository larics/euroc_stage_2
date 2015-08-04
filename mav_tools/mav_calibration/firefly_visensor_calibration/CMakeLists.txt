cmake_minimum_required(VERSION 2.8.3)
project(firefly_visensor_calibration)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_executable(firefly_visensor_calibration_node src/firefly_visensor_calibration_node.cpp)


add_dependencies(firefly_visensor_calibration_node ${catkin_EXPORTED_TARGETS})
 target_link_libraries(firefly_visensor_calibration_node
   ${catkin_LIBRARIES}
)
