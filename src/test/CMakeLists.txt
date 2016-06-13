
add_executable(aug_cov_test aug_cov_test.cpp)
add_dependencies(aug_cov_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(aug_cov_test ${catkin_LIBRARIES})

#add_executable(path_test path_test.cpp)
#add_dependencies(path_test ${catkin_EXPORTED_TARGETS})
#target_link_libraries(path_test ${catkin_LIBRARIES})

#catkin_add_gtest(test_path_planning path_planning.cpp)
#target_link_libraries(test_path_planning ${catkin_LIBRARIES} yaml-cpp)

catkin_add_gtest(test_polynomial polynomial.cpp)
target_link_libraries(test_polynomial ${catkin_LIBRARIES} mav_planning_utils_rpoly)

catkin_add_gtest(test_polynomial_optimization polynomial_optimization.cpp)
target_link_libraries(test_polynomial_optimization ${catkin_LIBRARIES} ${PROJECT_NAME} ${PROJECT_NAME}_timing ${PROJECT_NAME}_rpoly)

catkin_add_gtest(test_trajectory_sampling trajectory_sampling.cpp)
target_link_libraries(test_trajectory_sampling ${catkin_LIBRARIES} ${PROJECT_NAME} ${PROJECT_NAME}_timing ${PROJECT_NAME}_rpoly)

catkin_add_gtest(test_polynomial_timing_evaluation polynomial_timing_evaluation.cpp)
target_link_libraries(test_polynomial_timing_evaluation ${catkin_LIBRARIES} ${PROJECT_NAME} ${PROJECT_NAME}_timing ${PROJECT_NAME}_rpoly)