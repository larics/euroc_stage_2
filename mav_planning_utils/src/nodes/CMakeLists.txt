#add_executable(polynomial_sampling_node polynomial_sampling.cpp)
#add_dependencies(polynomial_sampling_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(polynomial_sampling_node ${catkin_LIBRARIES} mav_planning_utils_timing mav_planning_utils_rpoly)

#add_executable(polynomial_sampling_node_new polynomial_sampling_new.cpp)
#add_dependencies(polynomial_sampling_node_new ${catkin_EXPORTED_TARGETS})
#target_link_libraries(polynomial_sampling_node_new ${catkin_LIBRARIES} mav_planning_utils_timing mav_planning_utils_rpoly)

add_executable(example_node example_node.cpp)
target_link_libraries(example_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${PROJECT_NAME}_timing ${PROJECT_NAME}_rpoly yaml-cpp)
add_dependencies(example_node ${catkin_EXPORTED_TARGETS})

add_executable(trajectory_sampling_node trajectory_sampling.cpp)
target_link_libraries(trajectory_sampling_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${PROJECT_NAME}_timing ${PROJECT_NAME}_rpoly yaml-cpp)
add_dependencies(trajectory_sampling_node ${catkin_EXPORTED_TARGETS})
