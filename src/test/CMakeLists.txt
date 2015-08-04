if (CATKIN_ENABLE_TESTING)
  find_package(eigen_checks REQUIRED)
  find_package(glog_catkin REQUIRED)
  catkin_add_gtest(test_rotations test_rotations.cpp)
  target_link_libraries(test_rotations ${catkin_LIBRARIES} ${eigen_checks_LIBRARIES} ${glog_catkin_LIBRARIES})
  SET_TARGET_PROPERTIES(test_rotations PROPERTIES COMPILE_FLAGS "-std=c++11")
endif()

