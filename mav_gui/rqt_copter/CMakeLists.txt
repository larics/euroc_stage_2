cmake_minimum_required(VERSION 2.8.3)
project(rqt_copter)

find_package(catkin REQUIRED COMPONENTS rospy rqt_gui rqt_gui_py std_msgs geometry_msgs)

catkin_python_setup()
catkin_package(
   CATKIN_DEPENDS rospy rqt_gui rqt_gui_py std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

