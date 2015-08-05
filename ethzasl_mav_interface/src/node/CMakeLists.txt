add_executable(mav_interface_node node.cpp)
add_dependencies(mav_interface_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(mav_interface_node ${catkin_LIBRARIES} ${PROJECT_NAME})
SET_TARGET_PROPERTIES(mav_interface_node PROPERTIES COMPILE_FLAGS "-std=c++11")
