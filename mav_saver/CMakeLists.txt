cmake_minimum_required(VERSION 2.8.3)
project(mav_saver)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  asctec_communication_interface
  cmake_modules
  diagnostic_updater
  eigen_checks
  ethzasl_mav_interface
  mav_msgs
  nav_msgs
  nodelet
  octomap_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  vehicle_monitor_library
  kill_switch_library
)

find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS}
   				    ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package(
  DEPENDS Eigen Boost
  CATKIN_DEPENDS roscpp sensor_msgs tf actionlib diagnostic_updater cmake_modules mav_msgs nav_msgs nodelet octomap_msgs ethzasl_mav_interface vehicle_monitor_library std_msgs kill_switch_library
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES
)

add_executable(mav_saver_node src/mav_saver_node.cpp
							  src/mav_saver.cpp
							  src/safety_pose_publisher.cpp)
add_dependencies(mav_saver_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(mav_saver_node ${OCTOMAP_LIBRARIES} 
                                     ${catkin_LIBRARIES})
                                     
SET_TARGET_PROPERTIES(mav_saver_node PROPERTIES COMPILE_FLAGS "-std=c++11")