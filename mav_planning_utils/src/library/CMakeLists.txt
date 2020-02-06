#add_library(rviz_drawing rviz_drawing.cpp)

add_library(${PROJECT_NAME} 
  trajectory_types.cpp 
  motion_defines.cpp 
  ros_trajectory_interface.cpp
  trajectory_sampling.cpp
)  
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_library(${PROJECT_NAME}_timing timing.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)

add_library(${PROJECT_NAME}_rpoly rpoly.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
