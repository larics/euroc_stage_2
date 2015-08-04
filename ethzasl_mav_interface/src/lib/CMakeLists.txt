add_library(${PROJECT_NAME}  
  common_data_publisher_autopilot.cpp 
  common_data_publisher_trinity.cpp
  attitude_command_subscriber_trinity.cpp
  mav_interface.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11")