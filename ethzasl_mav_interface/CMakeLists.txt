cmake_minimum_required(VERSION 2.8.3)
project(ethzasl_mav_interface)
find_package(catkin REQUIRED COMPONENTS
  actionlib
  asctec_communication_interface
  cmake_modules
  diagnostic_updater
  dynamic_reconfigure
  eigen_checks
  mav_msgs
  nav_msgs
  nodelet
  roscpp
  sensor_msgs
  tf
)

find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

#generate_dynamic_reconfigure_options(cfg/HLInterface.cfg cfg/SSDK.cfg)

catkin_package(
  DEPENDS Eigen Boost
  CATKIN_DEPENDS roscpp sensor_msgs tf dynamic_reconfigure actionlib diagnostic_updater cmake_modules mav_msgs nav_msgs nodelet
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES
)

add_subdirectory(src/lib)
add_subdirectory(src/node)
add_subdirectory(src/nodelet)
add_subdirectory(src/test)

install(FILES src/nodelet/ethzasl_mav_interface_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
